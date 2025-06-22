from abc import abstractmethod
from typing import Optional

from tmtccmd import BackendBase
from tmtccmd.com_if import ComInterface
from tmtccmd.config import TmTcCfgHookBase, TmtcDefinitionWrapper, default_json_path
from tmtccmd.util import ObjectIdDictT, RetvalDictT


class CommonFsfwHookBase(TmTcCfgHookBase):
    def __init__(self, json_cfg_path: Optional[str] = None):
        if json_cfg_path is None:
            json_cfg_path = default_json_path()
        super().__init__(json_cfg_path=json_cfg_path)

    @abstractmethod
    def get_tmtc_definitions(self) -> TmtcDefinitionWrapper:
        pass

    @abstractmethod
    def assign_communication_interface(self, com_if_key: str) -> Optional[ComInterface]:
        pass

    def perform_mode_operation(self, tmtc_backend: BackendBase, mode: int):
        print("No custom mode operation implemented")

    def get_object_ids(self) -> ObjectIdDictT:
        from common_tmtc.config.object_ids import get_object_ids

        return get_object_ids()

    def get_retval_dict(self) -> RetvalDictT:
        return self.get_retval_dict()
