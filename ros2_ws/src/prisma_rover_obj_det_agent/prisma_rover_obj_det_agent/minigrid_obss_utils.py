from prisma_rover_obj_det_agent.textutils import *
from prisma_rover_obj_det_agent.dictlist import DictList

def preprocess_minigrid_dict_obs(obs, vocab, length):
    #print(obs["image"].shape)
    return {
        "image": np.array(obs["image"]),
        "text": preprocess_texts("[begin] " + obs["mission"] + " [end]", vocab, length)
    }

def preprocess_minigrid_dict_obss(obss, device):
    print(obss["image"].shape)
    return DictList({
        "image": torch.tensor(np.array([obs["image"] for obs in obss], device=device, dtype=torch.float))
        #"mission": [obs["mission"] for obs in obss]
    })

def convert_dict_to_pytorch_tensors(dict, device):
    new_dict = {}
    for key in dict:
        if len(dict["image"].shape) < 4:
            new_dict[key] = torch.tensor(np.expand_dims(dict[key], axis=0), device=device)
        else:
            new_dict[key] = torch.tensor(dict[key], device=device)
    return new_dict

def convert_dict_to_pytorch_timeseries_tensors(dict, device):
    new_dict = {}
    for key in dict:
        if len(dict["image"].shape) < 4:
            new_dict[key] = torch.tensor(np.expand_dims(dict[key], axis=[0, 1]), device=device)
        else:
            new_dict[key] = torch.tensor(dict[key], device=device)
    return new_dict

def preprocess_minigrid_dict_obs_to_tensor(obs, vocab, length, device):
    #print(obs["mission"])
    text = np.array([preprocess_texts("[begin] " + mission + " [end]", vocab, length) for mission in obs["mission"]])
    #print(text.shape)
    return {
        "image": torch.tensor(np.array(obs["image"]), device=device),
        "text": torch.tensor(text, device=device)
    }
