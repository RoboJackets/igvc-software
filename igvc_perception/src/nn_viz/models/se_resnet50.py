import torch
from torch.jit import ScriptModule


def load_se_resnet50(model_path):  # type: (str) -> ScriptModule
    model_path = str(model_path)

    model = torch.jit.load(model_path, map_location="cuda")

    return model
