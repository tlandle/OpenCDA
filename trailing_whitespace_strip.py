from __future__ import print_function
import os
import fileinput

from ecloud.globals import __ecloud__

# for line in (line.rstrip() for line in fileinput.FileInput("/home/jordan/eCloud/ecloud/core/common/vehicle_manager.py", inplace=1)):
#     print(line)
#     print(line, file=sys.stderr)

for (root,_,files) in os.walk(__ecloud__, topdown=True):
    for file in files:
        if file.endswith('.py'):
            print(f"stripping trailing whitespace in {file}")
            for line in (line.rstrip() for line in fileinput.FileInput(os.path.join(root, file), inplace=1)):
                print(line)
    