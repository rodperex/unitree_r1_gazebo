# Copyright 2026 Rodrigo Pérez-Rodríguez
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

"""The CRC of the low-level messages of unitree_sdk2 (LowCmd_, LowState_), with the SDK's own
code (unitree_sdk2py.utils.crc).

On Linux the SDK computes it with a compiled library (utils/lib/crc_<arch>.so), which its
setup.py does not install (it is in its repository, not in the installed package). Without it,
the SDK's own pure-Python CRC is used: the same result, slower (a few milliseconds per message).
"""


def crc_function(compiled_only=False):
    """A function msg -> CRC; with compiled_only, None if the compiled library is missing (for
    messages published too often for the pure-Python CRC)."""
    from unitree_sdk2py.utils import crc as sdk_crc
    try:
        return sdk_crc.CRC().Crc
    except OSError:
        if compiled_only:
            return None
    crc = object.__new__(sdk_crc.CRC)
    try:
        sdk_crc.CRC.__init__(crc)    # the formats of the messages, then the library: missing
    except OSError:
        pass
    crc.platform = 'python'           # anything but Linux: the SDK's pure-Python CRC
    return crc.Crc
