/****************************************************************************
 Copyright (c) 2023 Georgia Institute of Technology

 Permission is hereby granted, free of charge, to any person obtaining a copy
 of this software and associated documentation files (the "Software"), to deal
 in the Software without restriction, including without limitation the rights to
 use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies
 of the Software, and to permit persons to whom the Software is furnished to do so,
 subject to the following conditions:

 The above copyright notice and this permission notice shall be included in
 all copies or substantial portions of the Software.

 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 THE SOFTWARE.
****************************************************************************/


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
using ecloud::ObjectBuffer;
using ecloud::EdgeObstacleObject;
using ecloud::ObjectRequest;

std::atomic<int16_t> numCompletedVehicles_;
std::atomic<int16_t> numRepliedVehicles_;
std::atomic<int32_t> tickId_;
std::atomic<bool> pushedTick_;

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
std::vector<std::pair<int16_t, std::string>> serializedEdgeObjects_; // vehicleIdx, serializedObjBuffer


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

        repliedCars_[request->vehicle_index()] = true; // DEBUGGING

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
    
        const bool complete = ( numRepliedVehicles_.load() + numCompletedVehicles_.load() ) == numCars_;
        if ( complete && !pushedTick_ )
        {
            pushedTick_ = true;
            const int64_t lastClientDurationNS = request->duration_ns();
            simAPIClient_->PushTick( request->tick_id(), command_, lastClientDurationNS );
            LOG(INFO) << "tick " << request->tick_id() << " COMPLETE";
        }
        DLOG(INFO) << "Client_SendUpdate - received reply from vehicle " << request->vehicle_index() << " for tick id:" << request->tick_id();
        
        ServerUnaryReactor* reactor = context->DefaultReactor();
        reactor->Finish(Status::OK);
        return reactor;
    }

    // server can push WP *before* ticking world and client can fetch them before it ticks
    ServerUnaryReactor* Client_GetWaypoints(CallbackServerContext* context,
                               const WaypointRequest* request,
                               WaypointBuffer* buffer) override {

        for ( int i = 0; i < serializedEdgeObjects_.size(); i++ )
        {
            const std::pair<int16_t, std::string > wpPair = serializedEdgeWaypoints_[i];
            if ( wpPair.first == request->vehicle_index() )
            {
                buffer->set_vehicle_index(request->vehicle_index());
                WaypointBuffer waypointBuf;
                //LOG(INFO) << "Requesting vehicle " << request->vehicle_index() << " waypoints starting parse";
                const std::string buf = wpPair.second;
                wpBuf.ParseFromString(buf);
                //LOG(INFO) << "Requesting vehicle " << request->vehicle_index() << " waypoints parsed";
                for ( Waypoint wp : wpBuf.waypoint_buffer())
                {
                    Waypoint *p = buffer->add_waypoint_buffer();
                    p->CopyFrom(wp);
                    //LOG(INFO) << "Requesting vehicle " << request->vehicle_index() << " single waypoint copied";
                }
                //LOG(INFO) << "Requesting vehicle " << request->vehicle_index() << " all waypoints copied";
                break;
            }
        }
        //LOG(INFO) << "vehicle " << request->vehicle_index() << " waypoints sent";


        ServerUnaryReactor* reactor = context->DefaultReactor();
        reactor->Finish(Status::OK);
        return reactor;
    }

    //
    // Clients grab objects before tick. 
    //
    ServerUnaryReactor* Client_GetObjects(CallbackServerContext* context,
                               const ObjectRequest* request,
                               ObjectBuffer* buffer) override {

        for ( int i = 0; i < serializedEdgeObjects_.size(); i++ )
        {
            const std::pair<int16_t, std::string > objPair = serializedEdgeObjects_[i];
            if ( objPair.first == request->vehicle_id() )
            {
                buffer->set_vehicle_id(request->vehicle_id());
                ObjectBuffer objBuf;
                //LOG(INFO) << "Requesting vehicle " << request->vehicle_index() << " waypoints starting parse";
                const std::string buf = objPair.second;
                objBuf.ParseFromString(buf);
                //LOG(INFO) << "Requesting vehicle " << request->vehicle_index() << " waypoints parsed";
                for ( EdgeObstacleObject obj : objBuf.object())
                {
                    EdgeObstacleObject *p = buffer->add_object();
                    p->CopyFrom(obj);
                    //LOG(INFO) << "Requesting vehicle " << request->vehicle_index() << " single waypoint copied";
                }
                //LOG(INFO) << "Requesting vehicle " << request->vehicle_index() << " all waypoints copied";
                break;
            }
        }
        //LOG(INFO) << "vehicle " << request->vehicle_index() << " waypoints sent";


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
            const int16_t vIdx = numRegisteredVehicles_.load();
            reply->set_vehicle_index(vIdx);
            const std::string connection = absl::StrFormat("%s:%d", request->vehicle_ip(), request->vehicle_port());
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
            const int16_t vIdx = request->vehicle_index();
            reply->set_vehicle_index(vIdx);

            DLOG(INFO) << "RegisterVehicle - CARLA_UPDATE - vehicle_index: " << vIdx << " | actor_id: " << request->actor_id() << " | vid: " << request->vid();

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

        const int16_t replies = numRepliedVehicles_.load();
        LOG(INFO) << "received " << numRegisteredVehicles_.load() << " registrations";
        LOG(INFO) << "received " << replies << " replies with Carla data";
        const bool complete = ( replies == numCars_ );

        LOG_IF(INFO, complete ) << "REGISTRATION COMPLETE";
        if ( complete )
        {
            assert( vehState_ == VehicleState::REGISTERING && replies == pendingReplies_.size() );
            simAPIClient_->PushTick( TICK_ID_INVALID, command_, INVALID_TIME );
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

        pushedTick_ = false;
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

        //LOG(INFO) << "updated waypoints received";
        for ( WaypointBuffer wpBuf : edgeWaypoints->all_waypoint_buffers() )
        {   
            std::string serializedWPs;
            wpBuf.SerializeToString(&serializedWPs);
            const std::pair< int16_t, std::string > wpPair = std::make_pair( wpBuf.vehicle_index(), serializedWPs );
            serializedEdgeWaypoints_.push_back(wpPair);
            //LOG(INFO) << "updated waypoints for vehicle index " << wpBuf.vehicle_index();
        }
        //LOG(INFO) << "updated waypoints processed";

        ServerUnaryReactor* reactor = context->DefaultReactor();
        reactor->Finish(Status::OK);
        return reactor;
    }

    ServerUnaryReactor* Server_PushEdgeObjects(CallbackServerContext* context,
                               const EdgeObstacleObjects* edgeObjects,
                               Empty* empty) override {
        serializedEdgObjects_.clear();

        //LOG(INFO) << "updated waypoints received";
        for ( ObjectBuffer objBuf : edgeObjects->all_objects() )
        {   
            std::string serializedObjs;
            objBuf.SerializeToString(&serializedObjs);
            const std::pair< int16_t, std::string > objPair = std::make_pair( objBuf.vehicle_id(), serializedObjs );
            serializedEdgeWaypoints_.push_back(wpPair);
            //LOG(INFO) << "updated waypoints for vehicle index " << wpBuf.vehicle_index();
        }
        //LOG(INFO) << "updated waypoints processed";

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
