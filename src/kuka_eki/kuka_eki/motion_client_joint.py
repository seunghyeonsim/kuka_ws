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

#A1=“90.958824" A2="-64.273476" A3="119.457329" A4="-8.596158" A5="-55.124290" A6="5.086195"
target_1_axis = Axis(30, -70.164719, 100.771515, 10.816616, -52.345718, 119.909058)
target_2_axis = Axis(30.259180, -88.164719, 137.771515, 1.816616, -52.345718, 119.909058)

#172.31.1.147
eki_motion_client = EkiMotionClient("172.31.1.147")
eki_state_client = EkiStateClient("172.31.1.147")
eki_motion_client.connect()
eki_state_client.connect()

eki_motion_client.ptp(target_1_axis, 0.5)
eki_motion_client.ptp(target_2_axis, 0.5)
while True:
    print(eki_state_client.state().pos)
    print(eki_state_client.state().axis)

