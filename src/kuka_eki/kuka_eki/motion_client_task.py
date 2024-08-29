# Copyright 2021 Norwegian University of Science and Technology.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from kuka_eki.eki import EkiMotionClient
from kuka_eki.eki import EkiStateClient
from kuka_eki.krl import Axis, Pos

target_1_Pos = Pos(1203.366821, -960.509521, 600.978516, -120.460846, -31.048431, -86.819878)
target_2_Pos = Pos(1203.366821, -960.509521, 1000.978516, -120.460846, -31.048431, -86.819878)

#172.31.1.147
eki_motion_client = EkiMotionClient("172.31.1.147")
eki_state_client = EkiStateClient("172.31.1.147")
eki_motion_client.connect()
eki_state_client.connect()

eki_motion_client.ptp(target_1_Pos, 0.5)
eki_motion_client.ptp(target_2_Pos, 0.5)
while True:
    print(eki_state_client.state().pos)
    print(eki_state_client.state().axis)

