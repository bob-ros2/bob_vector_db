import os
import io
import uuid
import json
import base64
import logging
import json_embedder
from PIL import Image as PIL_Image
from qdrant_client import QdrantClient
from qdrant_client import models
from fastembed import TextEmbedding
from fastembed import ImageEmbedding
from typing import Union

class QdrantEmbedder(json_embedder.JsonEmbedder):
    """Qdrant JSON embedding interface.

    The __call__ function expects a string with a JSON dict containing the following attributes:
      collection: to be used collection name
      documents: list with document content, if also images are provided this should be the caption
      metadatas: list of metadata dictionaries used as payload
      images: optional list of image file names to embed along with the documents
      ids: optional list of id strings, if not provided automatically uuid4 are created
    """

    def __init__(self, model: str, **kwargs):
        """QdrantEmbedder constructor

        :param model: Model to be used for text embedding or for images a text and a vision model separated by a space
        :type model: str
        :**kwargs: Args to be passed to the QdrantClient contructor
        :type kwargs: Variable length argument list
        """

        if not len(kwargs):
            self.client = QdrantClient(host='localhost', port=6333)
        else:
            self.client = QdrantClient(**kwargs)

        self.model = model
        self.text_model = None
        self.image_model = None

    def __call__(self, j: str, media: list=None) -> None:
        """Performs the embedding into the Vector DB

        :param j: Parsable JSON string with embedding information 
        :type j: str
        """

        data = json.loads(j)

        if 'images' in data or media:
            self.embed_images(data, media)
        else:
            self.client.set_model(
                self.model or 'sentence-transformers/all-MiniLM-L6-v2')
            self.client.add(
                collection_name = data['collection'], 
                documents = data['documents'], 
                metadata = data['metadatas'], 
                ids = None if 'ids' not in data else data['ids'])

    def embed_images(self, data: dict, media: list=None):
        """Embed dictionary with embedding information.

        :param data: Dictionary with embedding information
        :type data: dict
        """

        self.model = self.model or "Qdrant/clip-ViT-B-32-text Qdrant/clip-ViT-B-32-vision"
        model = self.model.split() 
        assert len(model) == 2

        # CLIP text encoder
        self.text_model = self.text_model or TextEmbedding(model_name=model[0]) 
        # dimension of text embeddings, produced by CLIP text encoder (512)
        text_embeddings_size = self.text_model._get_model_description(model[0])["dim"]
        texts_embedded = list(self.text_model.embed([document for document in data['documents']]))

        # CLIP image encoder
        if media:
            if 'images' in data: 
                logging.warning(
                    "embed_images: image(s) are provided twice! Will take them from media list")
            data['images'] = media
        self.image_model = self.image_model or ImageEmbedding(model_name=model[1]) 
        # dimension of image embeddings, produced by CLIP image encoder (512)
        image_embeddings_size = self.image_model._get_model_description(model[1])["dim"]
        images_embedded = list(self.image_model.embed([image for image in data['images']]))

        if not self.client.collection_exists(data['collection']): # creating collection
            self.client.create_collection(
                collection_name = data['collection'],
                vectors_config = { # named vectors
                    "image": models.VectorParams(
                        size = image_embeddings_size, 
                        distance = models.Distance.COSINE),
                    "text": models.VectorParams(
                        size = text_embeddings_size, 
                        distance = models.Distance.COSINE),
                }
            )

        def to_base64(image: Union[str, PIL_Image.Image]) -> str:
            if isinstance(image, str):
                with open(image, "rb") as file:
                    return base64.b64encode(file.read()).decode('ascii')
            else:
                buffered = io.BytesIO()
                image.save(buffered, 
                    format=os.getenv('EMBED_PIL_B64FORMAT', 'JPEG'))
                return base64.b64encode(buffered.getvalue()).decode('ascii')

        def make_id(data: dict, idx: int) -> str:
            if 'ids' in data: return data['ids'][idx]
            return str(uuid.uuid4())

        def make_payload(data, idx):
            payload = data['metadatas'][idx]
            if os.getenv('EMBED_IMAGES_BASE64', '') in ['1','true'] \
                and 'image_base64' not in payload:
                payload['image_base64'] \
                    = to_base64(data['images'][idx])
                # if image is coming from image topic there is no origin file
                if not media and 'image_origin' not in payload:
                    payload['image_origin'] = data['images'][idx]
            return payload

        self.client.upload_points(
            collection_name = data['collection'],
            points = [
                models.PointStruct(
                    id = make_id(data, idx), # unique id of a point
                    vector = {
                        "text":  texts_embedded[idx], # embeded caption
                        "image": images_embedded[idx] # embeded image
                    },
                    payload = make_payload(data, idx)
                )
                for idx, _ in enumerate(data['metadatas'])
            ]
        )
