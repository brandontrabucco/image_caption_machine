import nltk, re, pprint
from nltk import word_tokenize
from urllib import request

class VocabularyWriter(object):

    def __init__(self, url):
        self.url = url
        self.download()

    def download(self):
        self.raw = request.urlopen(
            self.url).read().decode('utf8')
        self.tokenize()

    def tokenize(self):
        self.vocab = nltk.FreqDist(
            [w.lower() for w in word_tokenize(self.raw)])
        self.write()

    def write(self):



        with open("word.vocab", "w") as f:
            for v in self.vocab.most_common(1000):
                try:
                    f.write(v[0] + "\n")
                except UnicodeEncodeError:
                    pass

if __name__ == "__main__":
    writer = VocabularyWriter("http://www.gutenberg.org/files/2554/2554-0.txt")