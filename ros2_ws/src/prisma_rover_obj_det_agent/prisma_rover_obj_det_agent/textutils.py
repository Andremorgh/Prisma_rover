import re
import numpy as np
import torch
import json
import ast

class Vocabulary:
    """A mapping from tokens to ids with a capacity of `max_size` words.
    It can be saved in a `vocab.json` file."""

    def __init__(self, max_size):
        self.max_size = max_size
        self.vocab = {'[begin]': 1, '[end]': 2}
        self.reverse_vocab = {1: '[begin]', 2: '[end]'}

    def __getitem__(self, token):
        if not token in self.vocab.keys():
            if len(self.vocab) >= self.max_size:
                raise ValueError("Maximum vocabulary capacity reached")
            self.vocab[token] = len(self.vocab) + 1
            self.reverse_vocab[len(self.vocab)] = token
        return self.vocab[token]

    def get_token(self, index):
        return self.reverse_vocab[index]
    
    def save_to_json(self, name):
        with open(name, "w") as fp:
            json.dump(self.vocab, fp)
    
    def load_from_json(self, path):
        with open(path, "r") as fp:
            self.vocab = ast.literal_eval(fp.read())
        
    def n_words(self):
        return len(self.vocab)


def make_ngrams(sentences):
    x = []
    y = []
    for sentence in sentences:
        x.append(sentence[:-1])
        y.append(sentence[-1])
        for ngram in padded_ngrams(sentence):
            x.append(ngram[:-1])
            y.append(ngram[-1])
    return np.array(x), np.array(y)

'''
def preprocess_texts(texts, vocab, encoding_length=10):
    var_indexed_texts = []
    #max_text_len = 0
    for text in texts:
        tokens = re.findall("([a-z0-9’.'\[\]]+)", text.lower())
        var_indexed_text = np.array([vocab[token] for token in tokens])
        encoding = np.zeros(encoding_length, dtype=np.int32)
        encoding[encoding_length - len(var_indexed_text):encoding_length] = var_indexed_text[:]
        var_indexed_texts.append(encoding)
        #max_text_len = max(len(var_indexed_text), max_text_len)
    
    #indexed_texts = np.zeros((len(texts), max_text_len))

    #for i, indexed_text in enumerate(var_indexed_texts):
    #    indexed_texts[i, :len(indexed_text)] = indexed_text

    return np.array(var_indexed_texts)
'''
def preprocess_texts(text, vocab, encoding_length=10):
    #var_indexed_texts = []
    #max_text_len = 0
    tokens = re.findall("([a-z0-9’.'\[\]]+)", text.lower())
    var_indexed_text = np.array([vocab[token] for token in tokens])
    encoding = np.zeros(encoding_length, dtype=np.int32)
    encoding[encoding_length - len(var_indexed_text):encoding_length] = var_indexed_text[:]
    #var_indexed_texts.append(encoding)
    return encoding


def preprocess_texts_ngrams(texts, vocab, encoding_length=10):
    var_indexed_texts = []
    #max_text_len = 0

    for text in texts:
        tokens = re.findall("([a-z0-9’.'\[\]]+)", text.lower())
        var_indexed_text = np.array([vocab[token] for token in tokens])
        encoding = np.zeros(encoding_length, dtype=np.int32)
        encoding[encoding_length - len(var_indexed_text):encoding_length] = var_indexed_text[:]
        for sentence in padded_ngrams(encoding):
            var_indexed_texts.append(sentence)
        
    return np.array(var_indexed_texts)



def padded_ngrams(text):
    ret = []
    for i in range(np.count_nonzero(text)+1):
        k = np.zeros(len(text), dtype=np.int32)
        k[i:len(k)] = text[:len(text)-i]
        ret.append(k)
    return ret

def sentence_from_tokens(tokens, vocab):
    text = ""
    for i in tokens:
        if i != 0:
            text += vocab.get_token(i) + " "
    return text