import os
import torch.nn.functional as F
import torch
import torch.nn as nn
from torch.distributions.categorical import Categorical

from prisma_rover_obj_det_agent.minigrid_obss_utils import *

words = ["red", "blue", "purple", "green", "yellow", "grey", "ball", "key", "box"]
word_labels = [0, 0, 0, 0, 0, 0, 1, 1, 1]       # 0 - colors, 1 - objects

max_sentence_length = 15
vocab_max_size = 100
WORD_EMBEDDING_SIZE = 1024
calculate_gt_maps = False


def layer_init(layer, std=np.sqrt(2), bias_const=0.0):
    torch.nn.init.orthogonal_(layer.weight, std)
    torch.nn.init.constant_(layer.bias, bias_const)
    return layer

def layer_init_no_bias(layer, std=np.sqrt(2), bias_const=0.0):
    torch.nn.init.orthogonal_(layer.weight, std)
    return layer

class ObsMissionAttentionLayer(nn.Module):
    # We query the obs (image + hidden state of lstm) on the mission in order to change the focus on the different subtask w.r.t. what we see on the environment
    def __init__(self, word_embedding_size, d_k, obs_size):
        super(ObsMissionAttentionLayer, self).__init__()
        self.scores = None
        self.word_embedding_size = word_embedding_size
        self.obs_size = obs_size
        self.d_k = d_k
        self.key = layer_init(nn.Linear(self.obs_size, self.d_k))
        self.query = layer_init(nn.Linear(self.word_embedding_size, self.d_k))

    def get_word_attention_maps(self, state, mission, mask=None):
        queries = self.query(mission)
        keys = self.key(state)
        self.scores = torch.matmul(queries, keys.transpose(-1, -2)) / torch.sqrt(torch.tensor(self.d_k, dtype=torch.float32))
        attention_weights = F.softmax(self.scores, dim=-1)
        attention_entropies = -Categorical(probs=attention_weights).entropy()
        attention_entropies = attention_entropies.masked_fill(mask == 0, -1e9)
        word_attention_weights = F.softmax(attention_entropies, dim=-1)

        return attention_weights, word_attention_weights

    def img_to_text_attention(self, state, mission, mask=None):
        img_reshape = torch.reshape(state, (state.shape[0], state.shape[1], state.shape[2]*state.shape[3]))
        img = img_reshape.transpose(1, 2)
        attention_weights, word_attention_weights = self.get_word_attention_maps(img, mission, mask)

        attention_mask = torch.sum(attention_weights * word_attention_weights.unsqueeze(-1), dim=1)
        output = img * attention_mask.unsqueeze(-1)

        return output, attention_weights, word_attention_weights

class InternalMemoryModule(nn.Module):
    def __init__(self, obs_size, lstm_emb_size = 128, num_layers = 1):
        super(InternalMemoryModule, self).__init__()
        self.obs_size = obs_size
        self.lstm_emb_size = lstm_emb_size
        self.num_layers = num_layers
        self.state_lstm = nn.LSTM(self.obs_size, self.lstm_emb_size, batch_first=False, num_layers=self.num_layers)
        for name, param in self.state_lstm.named_parameters():
            if "bias" in name:
                nn.init.constant_(param, 0)
            elif "weight" in name:
                nn.init.orthogonal_(param, 1.0)

    def forward(self, state, done, internal_memory_hidden):
        batch_size = internal_memory_hidden[0].shape[1]
        state_reshape = torch.reshape(state, (-1, batch_size, self.obs_size))
        done = done.reshape((-1, batch_size))
        new_hidden = []
        for h, d in zip(state_reshape, done):
            h, internal_memory_hidden = self.state_lstm(
                h.unsqueeze(0),
                (
                    (1.0 - d).view(1, -1, 1) * internal_memory_hidden[0],
                    (1.0 - d).view(1, -1, 1) * internal_memory_hidden[1],
                ),
            )
            new_hidden += [h]
        
        state = torch.flatten(torch.cat(new_hidden), 0, 1)
        return state, internal_memory_hidden

def init_params(m):
    if type(m) == nn.Linear:
        m.weight.data.normal_(0, 1)
        m.weight.data *= 1 / torch.sqrt(m.weight.data.pow(2).sum(1, keepdim=True))
        if m.bias is not None:
            m.bias.data.fill_(0)

class Agent(nn.Module):
    def __init__(self, n_actions=7, text_recurrent=False, action_obs=False, num_layers=2, hidden_size=256, kernel_size=1, stride=1, padding=0):
        super().__init__()
        self.feature_maps_size = 7*7
        self.K_out = 128
        self.image_embedding_size = self.feature_maps_size*self.K_out
        self.d_k = 32
        
        self.text_recurrent=text_recurrent
        self.action_obs = action_obs
        self.hidden_size = hidden_size
        self.num_layers = num_layers
        
        self.obj_network = nn.Sequential(layer_init_no_bias(nn.Conv2d(1, 32, kernel_size=kernel_size, stride=stride, padding=padding, bias=False)),
                                         nn.BatchNorm2d(32),
                                         nn.ReLU(),
                                         layer_init_no_bias(nn.Conv2d(32, 64, kernel_size=kernel_size, stride=stride, padding=padding, bias=False)),
                                         nn.BatchNorm2d(64),
                                         nn.ReLU(),
                                         layer_init_no_bias(nn.Conv2d(64, 64, kernel_size=kernel_size, stride=stride, padding=padding, bias=False)),
                                         nn.BatchNorm2d(64),
                                         nn.ReLU(),
                                         layer_init_no_bias(nn.Conv2d(64, 64, kernel_size=kernel_size, stride=stride, padding=padding, bias=False)),
                                         nn.BatchNorm2d(64),
                                         nn.ReLU(),
                                         layer_init_no_bias(nn.Conv2d(64, self.K_out, kernel_size=kernel_size, stride=stride, padding=padding, bias=False)),
                                         nn.BatchNorm2d(self.K_out),
                                         nn.ReLU(),
        )

        self.color_network = nn.Sequential(layer_init_no_bias(nn.Conv2d(1, 32, kernel_size=kernel_size, stride=stride, padding=padding, bias=False)),
                                           nn.BatchNorm2d(32),
                                           nn.ReLU(),
                                           layer_init_no_bias(nn.Conv2d(32, 64, kernel_size=kernel_size, stride=stride, padding=padding, bias=False)),
                                           nn.BatchNorm2d(64),
                                           nn.ReLU(),
                                           layer_init_no_bias(nn.Conv2d(64, 64, kernel_size=kernel_size, stride=stride, padding=padding, bias=False)),
                                           nn.BatchNorm2d(64),
                                           nn.ReLU(),
                                           layer_init_no_bias(nn.Conv2d(64, 64, kernel_size=kernel_size, stride=stride, padding=padding, bias=False)),
                                           nn.BatchNorm2d(64),
                                           nn.ReLU(),
                                           layer_init_no_bias(nn.Conv2d(64, self.K_out, kernel_size=kernel_size, stride=stride, padding=padding, bias=False)),
                                           nn.BatchNorm2d(self.K_out),
                                           nn.ReLU(),
        )
        
        self.combined_network = nn.Sequential(layer_init_no_bias(nn.Conv2d(self.K_out*2, self.K_out, kernel_size=kernel_size, stride=stride, padding=padding, bias=False)),
                                              nn.BatchNorm2d(self.K_out),
                                              nn.ReLU())
        
        self.conv_embedding_network = nn.Sequential(layer_init_no_bias(nn.Conv2d(self.K_out, 32, kernel_size=3, stride=1, padding=1, bias=False)),
                                           nn.BatchNorm2d(32),
                                           nn.ReLU(),
                                           layer_init_no_bias(nn.Conv2d(32, 64, kernel_size=3, stride=1, padding=1, bias=False)),
                                           nn.BatchNorm2d(64),
                                           nn.ReLU(),
                                           layer_init_no_bias(nn.Conv2d(64, 64, kernel_size=3, stride=1, padding=1, bias=False)),
                                           nn.BatchNorm2d(64),
                                           nn.ReLU(),
                                           layer_init_no_bias(nn.Conv2d(64, self.K_out, kernel_size=3, stride=1, padding=1, bias=False)),
                                           nn.BatchNorm2d(self.K_out),
                                           nn.ReLU(),
        )
    
        self.word_embedding_size = WORD_EMBEDDING_SIZE
        self.text_dim = self.word_embedding_size#128
        self.GRU_hidden_size = 256

        self.word_embedding = nn.Sequential(nn.Embedding(vocab_max_size, self.word_embedding_size), nn.LayerNorm(self.word_embedding_size))

        self.film_blocks = []

        self.obs_mission_attention_obs_dim = self.K_out
        
        self.obj_attention = ObsMissionAttentionLayer(word_embedding_size=self.text_dim, d_k=self.d_k, obs_size=self.obs_mission_attention_obs_dim)
        self.color_attention = ObsMissionAttentionLayer(word_embedding_size=self.text_dim, d_k=self.d_k, obs_size=self.obs_mission_attention_obs_dim)

        self.internal_memory_input_size = self.image_embedding_size
        self.internal_memory_hidden_size = self.actor_critic_in_dim = 128
        self.internal_memory = InternalMemoryModule(obs_size=self.internal_memory_input_size, lstm_emb_size=self.internal_memory_hidden_size)

        
        self.text_rnn = nn.GRU(self.text_dim, self.GRU_hidden_size, batch_first=True, bidirectional=False)
        
        self.actor = nn.Sequential(layer_init(nn.Linear(self.actor_critic_in_dim + self.GRU_hidden_size, 64)),
                                   nn.Tanh(),
                                   layer_init(nn.Linear(64, n_actions)))
        self.critic = nn.Sequential(layer_init(nn.Linear(self.actor_critic_in_dim + self.GRU_hidden_size, 64)),
                                    nn.Tanh(),
                                    layer_init(nn.Linear(64, 1)))

        self.obj_text_attention_weights = None
        
        self.obj_word_attention_weights = None 
    def get_word_embedding(self, text):
        return self.word_embedding(text)

    def get_states(self, x, state_hidden, done, mask=None, **kwargs):
        img, mission = x['image'], x['text']
        
        emb = self.word_embedding(mission)
        img_for_conv = img.transpose(1, 3).transpose(2, 3).float()
        
        obj_obs = img_for_conv[:, 0].unsqueeze(1)
        
        color_obs = img_for_conv[:, 1].unsqueeze(1)
        
        obj_conv = self.obj_network(obj_obs)
        color_conv = self.color_network(color_obs)

        _, self.obj_text_attention_weights, self.obj_word_attention_weights = self.obj_attention.img_to_text_attention(obj_conv, emb, mask)
        obj_attention_mask = torch.sum(self.obj_text_attention_weights * self.obj_word_attention_weights.unsqueeze(-1), dim=1)

        _, self.color_text_attention_weights, self.color_word_attention_weights = self.color_attention.img_to_text_attention(color_conv, emb, mask)
        color_attention_mask = torch.sum(self.color_text_attention_weights * self.color_word_attention_weights.unsqueeze(-1), dim=1)

        total_attention_map = obj_attention_mask + color_attention_mask

        combined = self.combined_network(torch.cat((obj_conv, color_conv), dim=1))
        img_reshape = torch.reshape(combined, (combined.shape[0], combined.shape[1], combined.shape[2]*combined.shape[3]))
        img_reshape = img_reshape.transpose(1, 2)
        output = img_reshape + (img_reshape * total_attention_map.unsqueeze(-1))
        
        output, state_hidden = self.internal_memory(output.reshape((output.shape[0], -1)), done, state_hidden)

        _, rnn_embeddings = self.text_rnn(emb)
        return output, state_hidden, emb, rnn_embeddings.squeeze(0)

    def get_value(self, x, prev_action, state_hidden, done, mask=None, **kwargs):
        hidden, _, _, rnn_embeddings = self.get_states(x, prev_action, state_hidden, done, mask)
        return self.critic(torch.cat((hidden, rnn_embeddings), dim=-1))

    def get_action_and_value(self, x, state_hidden, done, action=None, mask=None, **kwargs):
        hidden, state_hidden, context, rnn_embeddings = self.get_states(x, state_hidden, done, mask)
        logits = self.actor(torch.cat((hidden, rnn_embeddings), dim=-1))
        probs = Categorical(logits=logits)
        task_finished = None
        if action is None:
            action = probs.sample()

        return action, probs.log_prob(action), probs.entropy(), self.critic(torch.cat((hidden, rnn_embeddings), dim=-1)), state_hidden, task_finished
    

