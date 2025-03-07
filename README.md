# ROS Package [bob_vector_db](https://github.com/bob-ros2/bob_vector_db)

This package integrates a Vector DB into ROS which can be accessed via ROS topics.

**Available Features**

* Simple ROS String topic JSON interface
* Embed text into a Chroma or Qdrant Vector DB including additional payload data
* Embed ROS sensor_msgs/msg/Image
 along with payload data into Qdrant Vector database
* Configurable model for the embeddings
* Multimodal vector embedding (Qdrant)
* Query the DB and return the results

## ROS Node EMBEDDER

This ROS node subscribes to an /embed String topic to receive JSON data with 
embedding messages. It embeds the delivered data into the configured Vector DB. 
A Qdrant Vector DB is the default DB, optionally a Chroma DB can be used.

By default Sentence Transformers `all-MiniLM-L6-v2` model will be used to 
create the embeddings.

Related Qdrant links:
- [https://qdrant.tech/](https://qdrant.tech/)
- [https://github.com/qdrant/fastembed](https://github.com/qdrant/fastembed)
- [https://qdrant.tech/documentation/embeddings](https://qdrant.tech/documentation/embeddings)
- [Supported Models](https://qdrant.github.io/fastembed/examples/Supported_Models/#supported-text-embedding-models)

Related Chroma links:
- [https://www.trychroma.com/](https://www.trychroma.com/)
- [https://docs.trychroma.com/](https://docs.trychroma.com/)

### Embedding data format

The JSON data which is received by the String topic has to contain the 
following fields. (for Qdrant the ids are optional) 

*Embed text*
```json
{
  "collection": "xfiles", 
  "documents": ["some story text","text about something strange"], 
  "metadatas": [{"title":"The end"}, {"title":"Dark star"}], 
  "ids": ["id1","id2"]
}
```

*Embed text + image (Qdrant only)*
```json
{
  "collection": "movie_cover", 
  "documents": ["some story text","text about something strange"], 
  "metadatas": [{"title":"The end"}, {"title":"Dark star"}], 
  "images": ["/path/to/image_id1.jpg","/path/to/image_id2.jpg"],
  "ids": ["id1","id2"]
}
```

**Image embedding environment variables**

Enable / disable automatic storing of the image in Base64 format into payload data:

>**EMBED_IMAGES_BASE64** Default: 0

Used image format when embedding from [sensor_msgs/msg/Image](https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Image.html) or [bob_msgs/msg/STTImage](https://github.com/bob-ros2/bob_msgs/blob/main/msg/TTImage.msg):

>**EMBED_PIL_B64FORMAT** Default: JPEG

### Usage
```bash
# in order to work some of the below examples expects a running Qdrant or Chroma Vector DB

# start embedder node, by default use Qdrant Vector DB running on localhost
ros2 run bob_vector_db embedder

# start embedder node using Chroma Vector DB running on localhost
EMBED_USE_CHROMA=1 Ros2 run bob_vector_db embedder

# run in debug mode, this will also set Python logging to level DEBUG
ros2 run bob_vector_db embedder --ros-args --log-level DEBUG

# this works without a Qdrant server
ros2 run bob_vector_db embedder --ros-args -p path:=/home/ros/qdrant_data

# this works without a Chroma server
ros2 run bob_vector_db embedder --ros-args -p path:=/home/ros/chroma_data -p use_chroma:=true

# Publish from shell an embed order with a single item.
ros2 topic pub --once embed std_msgs/msg/String 'data: "{\"collection\":\"xfiles\", \"documents\":[\"Bobs ROS nodes are a collection of NLP and LLM tools for ROS\"], \"metadatas\": [{\"author\":\"bob\"}], \"ids\":[\"id1\"]}"'

# Embed one or more image together with the text representation
# When embedding images vectors for both, the text and the image, are produced and stored into the Qdrant DB
ros2 topic pub --once embed std_msgs/msg/String 'data: "{\"collection\":\"image_data\", \"documents\":[\"An animal from the animal farm in pink\"], \"images\": [\"/home/ros/ros2_ws/img/images/piglet.jpg\"], \"metadatas\": [{\"author\":\"bob\"}]}"'

# Start image/text embedder together with a topic terminal to enter image embed messages manually
# When setting EMBED_IMAGES_BASE64=1 in addtition to the path the image is stored in BASE64 format into payload key `image_base64`
# this works only with a Qdrant database
EMBED_IMAGES_BASE64=1 ros2 launch bob_vector_db embedder.launch.py terminal:=true
```

### Node Parameter

> **Parameter name**: default_collection\
> **Type**: string\
> **Description**: Default collection to be used if embedding from embed_raw or embed_image topic. Environment variable EMBED_DEFAULT_COLLECTION. Default: embed_raw

> **Parameter name**: host\
> **Type**: string\
> **Description**: Vector DB host name or ip address. Environment variable EMBED_HOST. Default: localhost

> **Parameter name**: location\
> **Type**: string\
> **Description**: Vector DB location. Can be empty or url. This parameter is only used by Qdrant. Environment variable EMBED_LOCATION. Default: ''

> **Parameter name**: model\
> **Type**: string\
> **Description**: To be used embedding model. For Chroma currently only text embedding is available. When using Qdrant and to embed images a text and a image model can be provided separated by a space. Supported models: Qdrant/clip-ViT-B-32-text Qdrant/clip-ViT-B-32-vision. Environment variable EMBED_MODEL. Default: ''

> **Parameter name**: path\
> **Type**: string\
> **Description**: Vector DB local path if using persistent storage. Environment variable EMBED_PATH. Default: ''

> **Parameter name**: port\
> **Type**: integer\
> **Description**: Vector DB port. Environment variable EMBED_PORT. Default: 6333 or 8000 (chroma db)

> **Parameter name**: use_chroma\
> **Type**: boolean\
> **Description**: Use Chroma Vector DB instead of a Qdrant Vector DB. Environment variable EMBED_USE_CHROMA. Default: 0

### Subscribed Topics

> ~embed ([std_msgs/msg/String](https://docs.ros.org/en/melodic/api/std_msgs/html/msg/String.html))\
Incoming JSON string with the embedding data.

> ~embed_raw ([std_msgs/msg/String](https://docs.ros.org/en/melodic/api/std_msgs/html/msg/String.html))\
Incoming String with raw data.

> ~embed_image ([sensor_msgs/msg/Image](https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Image.html))\
Incoming ROS Image to embed.

> ~embed_ttimage ([bob_msgs/msg/TTImage](https://github.com/bob-ros2/bob_msgs/blob/main/msg/TTImage.msg))\
Incoming TTImage with payload to embed.
