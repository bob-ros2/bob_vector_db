import json
import json_embedder
from typing import Any
import chromadb
from chromadb.utils import embedding_functions as ef

class ChromaEmbedder(json_embedder.JsonEmbedder):
    """Chroma JSON embedding interface.
    
    The __call__ function expects a string with a JSON dict containing the following attributes:
      collection: to be used collection name
      documents: list with document strings
      metadatas: list of metadata dictionaries
      ids: list of id strings
    """

    def __init__(self, model: str = None, **kwargs: Any):
        if not len(kwargs):
            self.client = chromadb.HttpClient(host="localhost", port="8000")
        else:
            if 'path' in kwargs:
                self.client = chromadb.PersistentClient(path=kwargs['path'])
            else:
                self.client = chromadb.HttpClient(**kwargs)
        self.ef = ef.SentenceTransformerEmbeddingFunction(
            model_name=model or 'sentence-transformers/all-MiniLM-L6-v2')

    def __call__(self, j: str, media: list=None) -> None:
        data = json.loads(j)
        collection = self.client.get_or_create_collection(
            name = data['collection'], 
            embedding_function = self.ef)
        collection.add(
            documents = data['documents'],
            metadatas = data['metadatas'],
            ids = data['ids'])
