#include <iostream>
#include <memory>
#include <string>
#include <atomic>
#include <mutex>
#include <condition_variable>
#include <thread>
#include <cassert>
#include <stdexcept>
#include <errno.h>
#include <csignal>
#include <unistd.h>
#include <chrono>

#include "absl/flags/flag.h"
#include "absl/flags/parse.h"
#include "absl/strings/str_format.h"
#include "absl/log/log.h"
#include "absl/log/flags.h"
#include "absl/log/initialize.h"
#include "absl/log/globals.h"

#include <grpcpp/ext/proto_server_reflection_plugin.h>
#include <grpcpp/grpcpp.h>
#include <grpcpp/health_check_service_interface.h>

#include <google/protobuf/util/time_util.h>
#include <google/protobuf/util/json_util.h>

#include "ecloud.grpc.pb.h"
#include "ecloud.pb.h"

//#include <glog/logging.h>

#define WORLD_TICK_DEFAULT_MS 50
#define SLOW_CAR_COUNT 0
#define SPECTATOR_INDEX 0
#define VERBOSE_PRINT_COUNT 5
#define MAX_CARS 512
#define INVALID_TIME 0
#define TICK_ID_INVALID -1
#define VEHICLE_UPDATE_BATCH_SIZE 32

#define ECLOUD_PUSH_BASE_PORT 50101
#define ECLOUD_PUSH_API_PORT 50061

ABSL_FLAG(uint16_t, port, 50051, "Sim API server port for the service");
ABSL_FLAG(uint16_t, minloglevel, static_cast<uint16_t>(absl::LogSeverityAtLeast::kInfo),
          "Messages logged at a lower level than this don't actually "
          "get logged anywhere");

using google::protobuf::util::TimeUtil;

using grpc::CallbackServerContext;
using grpc::Server;
using grpc::ServerBuilder;
using grpc::ServerUnaryReactor;
using grpc::Status;

using ecloud::Ecloud;
using ecloud::EcloudResponse;
using ecloud::VehicleUpdate;
using ecloud::Empty;
using ecloud::Tick;
using ecloud::Command;
using ecloud::VehicleState;
using ecloud::SimulationInfo;
using ecloud::RegistrationInfo;
using ecloud::WaypointBuffer;
using ecloud::Waypoint;
using ecloud::Transform;
using ecloud::Location;
using ecloud::Rotation;
using ecloud::LocDebugHelper;
using ecloud::AgentDebugHelper;
using ecloud::PlanerDebugHelper;
using ecloud::ClientDebugHelper;
using ecloud::Timestamps;
using ecloud::WaypointRequest;
using ecloud::EdgeWaypoints;

volatile std::atomic<int16_t> numCompletedVehicles_;
volatile std::atomic<int16_t> numRepliedVehicles_;
volatile std::atomic<int32_t> tickId_;

bool repliedCars_[MAX_CARS];
std::string carNames_[MAX_CARS];

bool init_;
bool isEdge_;
int16_t numCars_;
std::string configYaml_;
std::string application_;
std::string version_;

std::string simIP_;

VehicleState vehState_;
Command command_;

std::vector<std::pair<int16_t, std::string>> serializedEdgeWaypoints_; // vehicleIdx, serializedWPBuffer

absl::Mutex mu_;

volatile std::atomic<int16_t> numRegisteredVehicles_ ABSL_GUARDED_BY(mu_);
std::vector<std::string> pendingReplies_ ABSL_GUARDED_BY(mu_); // TODO: Move to a hashmap serialized protobuf allows differing message types in same vector

class PushClient
{
    public:
        explicit PushClient( std::shared_ptr<grpc::Channel> channel, std::string connection ) :
                            stub_(Ecloud::NewStub(channel)), connection_(connection) {}

        bool PushTick(int32_t tickId, Command command, int64_t lastClientDurationNS)
        {
            Tick tick;
            tick.set_tick_id(tickId);
            tick.set_command(command);

            LOG_IF(INFO, command == Command::END) << "pushing END";

            tick.set_last_client_duration_ns(lastClientDurationNS);

            grpc::ClientContext context;
            Empty empty;

            // The actual RPC.
            std::mutex mu;
            std::condition_variable cv;
            bool done = false;
            Status status;
            stub_->async()->PushTick(&context, &tick, &empty,
                            [&mu, &cv, &done, &status](Status s) {
                            status = std::move(s);
                            std::lock_guard<std::mutex> lock(mu);
                            done = true;
                            cv.notify_one();
                            });

            std::unique_lock<std::mutex> lock(mu);
            while (!done) {
                cv.wait(lock);
            }

            // Act upon its status.
            if (status.ok()) {
                return true;
            } else {
                std::cout << status.error_code() << ": " << status.error_message()
                << std::endl;
                return false;
            }
        }

    private:
        std::unique_ptr<Ecloud::Stub> stub_;
        std::string connection_;
};

// Logic and data behind the server's behavior.
class EcloudServiceImpl final : public Ecloud::CallbackService {
public:
    explicit EcloudServiceImpl() {
        if ( !init_ )
        {
            numCompletedVehicles_.store(0);
            numRepliedVehicles_.store(0);
            numRegisteredVehicles_.store(0);
            tickId_.store(0);

            vehState_ = VehicleState::REGISTERING;
            command_ = Command::TICK;

            numCars_ = 0;
            configYaml_ = "";
            isEdge_ = false;

            simIP_ = "localhost";

            const std::string connection = absl::StrFormat("%s:%d", simIP_, ECLOUD_PUSH_API_PORT );
            simAPIClient_ = new PushClient(grpc::CreateChannel(connection, grpc::InsecureChannelCredentials()), connection);

            vehicleClients_.clear();
            pendingReplies_.clear();

            init_ = true;
        }
    }

    ServerUnaryReactor* Server_GetVehicleUpdates(CallbackServerContext* context,
                               const Empty* empty,
                               EcloudResponse* reply) override {

        DLOG(INFO) << "Server_GetVehicleUpdates - deserializing updates.";

        const int16_t replies = pendingReplies_.size();
        for ( int i = 0; i < replies; i++ )
        {
            VehicleUpdate *update = reply->add_vehicle_update();
            const std::string msg = pendingReplies_.back();
            pendingReplies_.pop_back();
            update->ParseFromString(msg);
            LOG(INFO) << "update: vehicle_index - " << update->vehicle_index();

            if ( i == VEHICLE_UPDATE_BATCH_SIZE ) // keep from exhausting resources
                break;
        }

        DLOG(INFO) << "Server_GetVehicleUpdates - updates deserialized.";

        if ( pendingReplies_.size() == 0 )
            numRepliedVehicles_ = 0;
    
        ServerUnaryReactor* reactor = context->DefaultReactor();
        reactor->Finish(Status::OK);
        return reactor;
    }

    ServerUnaryReactor* Client_SendUpdate(CallbackServerContext* context,
                               const VehicleUpdate* request,
                               Empty* empty) override {

        if ( isEdge_ || request->vehicle_index() == SPECTATOR_INDEX || request->vehicle_state() == VehicleState::TICK_DONE || request->vehicle_state() == VehicleState::DEBUG_INFO_UPDATE )
        {
            std::string msg;
            request->SerializeToString(&msg);
            if ( isEdge_ || request->vehicle_state() == VehicleState::TICK_DONE || request->vehicle_state() == VehicleState::DEBUG_INFO_UPDATE )
            {
                // TODO: hashmap
                mu_.Lock();
                pendingReplies_.push_back(msg);
                mu_.Unlock();
            }
            else
            {
                assert( request->vehicle_index() == SPECTATOR_INDEX );
                pendingReplies_.push_back(msg);
            }
        }

        repliedCars_[request->vehicle_index()] = true;

        DLOG(INFO) << "Client_SendUpdate - received reply from vehicle " << request->vehicle_index() << " for tick id:" << request->tick_id();

        if ( request->vehicle_state() == VehicleState::TICK_DONE )
        {
            numCompletedVehicles_++;
            DLOG(INFO) << "Client_SendUpdate - TICK_DONE - tick id: " << tickId_ << " vehicle id: " << request->vehicle_index();
        }
        else if ( request->vehicle_state() == VehicleState::TICK_OK )
        {
            numRepliedVehicles_++;
        }
        else if ( request->vehicle_state() == VehicleState::DEBUG_INFO_UPDATE )
        {
            numCompletedVehicles_++;
            DLOG(INFO) << "Client_SendUpdate - DEBUG_INFO_UPDATE - tick id: " << tickId_ << " vehicle id: " << request->vehicle_index();
        }

        // BEGIN PUSH
        const int16_t replies_ = numRepliedVehicles_.load();
        const int16_t completions_ = numCompletedVehicles_.load();
        const bool complete_ = ( replies_ + completions_ ) == numCars_;

        LOG_IF(INFO, complete_ ) << "tick " << request->tick_id() << " COMPLETE";
        if ( complete_ )
        {
            const int64_t lastClientDurationNS = request->duration_ns();
            simAPIClient_->PushTick( request->tick_id(), command_, lastClientDurationNS );
        }

        ServerUnaryReactor* reactor = context->DefaultReactor();
        reactor->Finish(Status::OK);
        return reactor;
    }

    // server can push WP *before* ticking world and client can fetch them before it ticks
    ServerUnaryReactor* Client_GetWaypoints(CallbackServerContext* context,
                               const WaypointRequest* request,
                               WaypointBuffer* buffer) override {

        for ( int i = 0; i < serializedEdgeWaypoints_.size(); i++ )
        {
            const std::pair<int16_t, std::string > wpPair = serializedEdgeWaypoints_[i];
            if ( wpPair.first == request->vehicle_index() )
            {
                buffer->set_vehicle_index(request->vehicle_index());
                WaypointBuffer *wpBuf;
                wpBuf->ParseFromString(wpPair.second);
                for ( Waypoint wp : wpBuf->waypoint_buffer())
                {
                    Waypoint *p = buffer->add_waypoint_buffer();
                    p->CopyFrom(wp);
                }
                break;
            }
        }

        ServerUnaryReactor* reactor = context->DefaultReactor();
        reactor->Finish(Status::OK);
        return reactor;
    }

    ServerUnaryReactor* Client_RegisterVehicle(CallbackServerContext* context,
                               const RegistrationInfo* request,
                               SimulationInfo* reply) override {

        assert( configYaml_ != "" );

        if ( request->vehicle_state() == VehicleState::REGISTERING )
        {
            DLOG(INFO) << "got a registration update";

            mu_.Lock();
            reply->set_vehicle_index(numRegisteredVehicles_.load());
            const std::string connection = absl::StrFormat("%s:%d", request->vehicle_ip(), ECLOUD_PUSH_BASE_PORT + numRegisteredVehicles_.load() );
            PushClient *vehicleClient = new PushClient(grpc::CreateChannel(connection, grpc::InsecureChannelCredentials()), connection);
            vehicleClients_.push_back(std::move(vehicleClient));
            numRegisteredVehicles_++;
            mu_.Unlock();

            reply->set_test_scenario(configYaml_);
            reply->set_application(application_);
            reply->set_version(version_);

            DLOG(INFO) << "RegisterVehicle - REGISTERING - container " << request->container_name() << " got vehicle id: " << reply->vehicle_index();
            carNames_[reply->vehicle_index()] = request->container_name();
        }
        else if ( request->vehicle_state() == VehicleState::CARLA_UPDATE )
        {
            reply->set_vehicle_index(request->vehicle_index());

            DLOG(INFO) << "RegisterVehicle - CARLA_UPDATE - vehicle_index: " << request->vehicle_index() << " | actor_id: " << request->actor_id() << " | vid: " << request->vid();

            // TODO: Hashmap
            mu_.Lock();
            std::string msg;
            request->SerializeToString(&msg);
            pendingReplies_.push_back(msg);
            numRepliedVehicles_++;
            mu_.Unlock();
        }
        else
        {
            assert(false);
        }

        const int16_t replies_ = numRepliedVehicles_.load();
        LOG(INFO) << "received " << replies_ << " replies";
        const bool complete_ = ( replies_ == numCars_ );

        LOG_IF(INFO, complete_ ) << "REGISTRATION COMPLETE";
        if ( complete_ )
        {
            assert( vehState_ == VehicleState::REGISTERING && replies_ == pendingReplies_.size() );
            simAPIClient_->PushTick( TICK_ID_INVALID, command_, INVALID_TIME);
        }

        ServerUnaryReactor* reactor = context->DefaultReactor();
        reactor->Finish(Status::OK);
        return reactor;
    }

    ServerUnaryReactor* Server_DoTick(CallbackServerContext* context,
                               const Tick* request,
                               Empty* empty) override {
        for ( int i = 0; i < numCars_; i++ )
            repliedCars_[i] = false;

        numRepliedVehicles_ = 0;
        assert(tickId_ == request->tick_id() - 1);
        tickId_++;
        command_ = request->command();

        const auto now = std::chrono::system_clock::now();
        DLOG(INFO) << "received new tick " << request->tick_id() << " at " << std::chrono::duration_cast<std::chrono::milliseconds>(
            now.time_since_epoch()).count();

        const int32_t tickId = request->tick_id();
        for ( int i = 0; i < vehicleClients_.size(); i++ )
        {
            PushClient *v = vehicleClients_[i];
            std::thread t( &PushClient::PushTick, v, tickId, command_, INVALID_TIME );
            t.detach();
        }

        ServerUnaryReactor* reactor = context->DefaultReactor();
        reactor->Finish(Status::OK);
        return reactor;
    }

    ServerUnaryReactor* Server_PushEdgeWaypoints(CallbackServerContext* context,
                               const EdgeWaypoints* edgeWaypoints,
                               Empty* empty) override {
        serializedEdgeWaypoints_.clear();

        for ( WaypointBuffer wpBuf : edgeWaypoints->all_waypoint_buffers() )
        {   std::string serializedWPs;
            wpBuf.SerializeToString(&serializedWPs);
            const std::pair< int16_t, std::string > wpPair = std::make_pair( wpBuf.vehicle_index(), serializedWPs );
            serializedEdgeWaypoints_.push_back(wpPair);
        }

        ServerUnaryReactor* reactor = context->DefaultReactor();
        reactor->Finish(Status::OK);
        return reactor;
    }

    ServerUnaryReactor* Server_StartScenario(CallbackServerContext* context,
                               const SimulationInfo* request,
                               Empty* empty) override {
        vehState_ = VehicleState::REGISTERING;

        configYaml_ = request->test_scenario();
        application_ = request->application();
        version_ = request->version();
        numCars_ = request->vehicle_index(); // bit of a hack to use vindex as count
        isEdge_ = request->is_edge();
        // TODO: simIP_ = // always localhost for now

        assert( numCars_ <= MAX_CARS );
        DLOG(INFO) << "numCars_: " << numCars_;

        ServerUnaryReactor* reactor = context->DefaultReactor();
        reactor->Finish(Status::OK);
        return reactor;
    }

    ServerUnaryReactor* Server_EndScenario(CallbackServerContext* context,
                               const Empty* request,
                               Empty* reply) override {
        command_ = Command::END;

        LOG(INFO) << "pushing END";
        for ( int i = 0; i < vehicleClients_.size(); i++ )
            vehicleClients_[i]->PushTick(TICK_ID_INVALID, Command::END, INVALID_TIME); // don't thread --> block

        ServerUnaryReactor* reactor = context->DefaultReactor();
        reactor->Finish(Status::OK);
        return reactor;
    }

    private:

        std::vector< PushClient * > vehicleClients_;
        PushClient * simAPIClient_;
};

void RunServer(uint16_t port) {
    EcloudServiceImpl service;

    grpc::EnableDefaultHealthCheckService(true);
    grpc::reflection::InitProtoReflectionServerBuilderPlugin();
    ServerBuilder builder;
    // Listen on the given address without any authentication mechanism.
    const std::string server_address = absl::StrFormat("0.0.0.0:%d", port );
    builder.AddListeningPort(server_address, grpc::InsecureServerCredentials());
    std::cout << "server listening on port " << port << std::endl;
    // Register "service" as the instance through which we'll communicate with
    // clients. In this case it corresponds to an *synchronous* service.
    builder.RegisterService(&service);
    // Sample way of setting keepalive arguments on the server. Here, we are
    // configuring the server to send keepalive pings at a period of 10 minutes
    // with a timeout of 20 seconds. Additionally, pings will be sent even if
    // there are no calls in flight on an active HTTP2 connection. When receiving
    // pings, the server will permit pings at an interval of 10 seconds.
    builder.AddChannelArgument(GRPC_ARG_KEEPALIVE_TIME_MS,
                                10 * 60 * 1000 /*10 min*/);
    builder.AddChannelArgument(GRPC_ARG_KEEPALIVE_TIMEOUT_MS,
                                20 * 1000 /*20 sec*/);
    builder.AddChannelArgument(GRPC_ARG_KEEPALIVE_PERMIT_WITHOUT_CALLS, 1);
    builder.AddChannelArgument(
        GRPC_ARG_HTTP2_MIN_RECV_PING_INTERVAL_WITHOUT_DATA_MS,
        10 * 1000 /*10 sec*/);
    // Finally assemble the server.
    std::unique_ptr<Server> server(builder.BuildAndStart());

    // Wait for the server to shutdown. Note that some other thread must be
    // responsible for shutting down the server for this call to ever return.
    server->Wait();
}

int main(int argc, char* argv[]) {

    // 2 - std::cout << "ABSL: ERROR - " << static_cast<uint16_t>(absl::LogSeverityAtLeast::kError) << std::endl;
    // 1 - std::cout << "ABSL: WARNING - " << static_cast<uint16_t>(absl::LogSeverityAtLeast::kWarning) << std::endl;
    // 0 - std::cout << "ABSL: INFO - " << static_cast<uint16_t>(absl::LogSeverityAtLeast::kInfo) << std::endl;

    absl::ParseCommandLine(argc, argv);
    //absl::InitializeLog();

    std::thread server = std::thread(&RunServer,absl::GetFlag(FLAGS_port));
    
    absl::SetMinLogLevel(static_cast<absl::LogSeverityAtLeast>(absl::GetFlag(FLAGS_minloglevel)));

    server.join();

    return 0;
}
