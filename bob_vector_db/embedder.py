#!/usr/bin/env python3
#
# Copyright 2024 BobRos
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
#

import os
import json
import logging
import traceback
import rclpy
from rclpy.node import Node
from rclpy.logging import LoggingSeverity
from rcl_interfaces.msg import ParameterDescriptor
from std_msgs.msg import String
from sensor_msgs.msg import Image
from bob_msgs.msg import TTImage
from typing import Union

class EmbedderNode(Node):
    """Basic Embedder ROS Node.
    This ROS node subscribes to a std_msgs/msg/String topic to receive JSON data with embedding messages. 
    Additionaly, when using a Qdrant vector Database, it's possible to embed Images along with text and 
    Payload data from a sensor_msgs/msg/Image or bob_msgs/msg/STTImage. 

    It embeds the delivered data into the configured Vector DB. A Qdrant Vector DB is the default DB. 
    To embed into a Chroma DB set parameter 'use_chroma' to true. See ROS parameter for further configuration. 
    The JSON data has to contain the following fields. (for Qdrant the ids are optional).

    `Example of a text embedding`::

        {
            "collection": "stories", 
            "documents": [
                "some story text",
                "text abou something strange"
            ], 
            "metadatas": [
                {"title":"The end"}, 
                {"title":"Dark star"}
            ], 
            "ids":[
                "id1",
                "id2"
            ]
        }

    `Example of a text + image embedding`::

        {
            "collection": "movie_cover", 
            "documents": [
                "some story text",
                "text about something strange"
            ], 
            "metadatas": [
                {"title":"The end"}, 
                {"title":"Dark star"}
            ], 
            "images": [
                "/path/to/cover_image_id1.jpg", 
                "/path/to/cover_image_id2.jpg"
            ], "ids":[
                "id1",
                "id2"
            ]
        }

    For the image embedding there eixst following environment variables:

    Setting this to 1 will enable automatic storing of the image in Base64 format into the payload data:
        EMBED_IMAGES_BASE64 Default: 0
        
    When embedding from sensor_msgs/msg/Image or bob_msgs/msg/STTImage the image format can be configured
        EMBED_PIL_B64FORMAT  Default: JPEG

    To embed also sensor_msgs/msg/Image or bob_msgs/msg/STTImage see README.md for further information.
    """
    def __init__(self):
        super().__init__('embedder')

        logging.basicConfig(
            level = (logging.DEBUG
                if self.get_logger().get_effective_level() == LoggingSeverity.DEBUG \
                else logging.INFO),
            format="[%(levelname)s] [%(asctime)s.] [%(name)s]: %(message)s",
            datefmt="%s")

        self.declare_parameters(
            namespace='',
            parameters=[

            ('model', os.getenv('EMBED_MODEL', ''), 
            ParameterDescriptor(description=
            'To be used embedding model. '
            'For Chroma currently only text embedding is available. '
            'When using Qdrant and to embed images and/or text and a image '
            'model can be provided separated by a space. '
            'Supported models: Qdrant/clip-ViT-B-32-text Qdrant/clip-ViT-B-32-vision. '
            "Environment variable EMBED_MODEL. Default: ''")),

            ('host', os.getenv('EMBED_HOST', 'localhost'), 
            ParameterDescriptor(description=
            'Vector DB host name or ip address. '
            'Environment variable EMBED_HOST. Default: localhost')),

            ('port', int(os.getenv('EMBED_PORT', ('8000' 
                if int('0' or os.getenv('EMBED_USE_CHROMA')) 
                else '6333' ))), 
            ParameterDescriptor(description=
            'Vector DB port. Environment variable EMBED_PORT. '
            'Default: 6333 or 8000 (chroma db)')),

            ('path', os.getenv('EMBED_PATH', ''), 
            ParameterDescriptor(description=
            'Vector DB local path if using persistent storage. '
            "Environment variable EMBED_PATH. Default: ''")),

            ('location', os.getenv('EMBED_LOCATION', ''), 
            ParameterDescriptor(description=
            'Vector DB location. Can be empty or url. '
            'This parameter is only used by Qdrant. '
            "Environment variable EMBED_LOCATION. Default: ''")),

            ('use_chroma', bool(int('0' or os.getenv('EMBED_USE_CHROMA', '0'))), 
            ParameterDescriptor(description=
            'Use Chroma Vector DB instead of a Qdrant Vector DB. '
            "Environment variable EMBED_USE_CHROMA. Default: 0")),

            ('default_collection', os.getenv('EMBED_DEFAULT_COLLECTION', 'embed_raw'), 
            ParameterDescriptor(description=
            'Default collection to be used if embedding from embed_raw or embed_image topic. '
            "Environment variable EMBED_DEFAULT_COLLECTION. Default: embed_raw"))
        ])

        self.model = self.get_parameter('model').value or None
        self.host = self.get_parameter('host').value
        self.port = self.get_parameter('port').value  
        self.path = self.get_parameter('path').value
        self.location = self.get_parameter('location').value

        if self.get_parameter('use_chroma').value:
            self.embed = self.init_chroma()
        else:
            self.embed = self.init_qdrant()

        # subscribe to embed topic
        self.sub = self.create_subscription(
            String, 'embed', self.embed_callback, 1000)

        # subscribe to embed_raw topic
        self.sub_raw = self.create_subscription(
            String, 'embed_raw', self.embed_raw_callback, 1000)

        # subscribe to embed_image topic
        self.sub_image = self.create_subscription(
            Image, 'embed_image', self.embed_image_callback, 1000)

        # subscribe to embed_ttimage topic
        self.sub_ttimage = self.create_subscription(
            TTImage, 'embed_ttimage', self.embed_ttimage_callback, 1000)

        self.get_logger().info('Init done, waiting for embed messages')

    def init_qdrant(self):
        """Init Qdrant embedder
        """
        from qdrant_embedder import QdrantEmbedder
        self.get_logger().info('using Qdrant DB')
        if self.location:
            return QdrantEmbedder(
                self.model, prefer_grpc=True, location=self.location)
        elif self.path:
            return QdrantEmbedder(
                self.model, prefer_grpc=True, path=self.path)
        elif self.host and self.port:
            return QdrantEmbedder(
                self.model, prefer_grpc=True, host=self.host, port=self.port)
        elif self.host:
            return QdrantEmbedder(
                self.model, prefer_grpc=True, host=self.host)
        else:
            return QdrantEmbedder(
                self.model, prefer_grpc=True)

    def init_chroma(self):
        """Init Chroma embedder
        """
        from chroma_embedder import ChromaEmbedder
        self.get_logger().info('using Chroma DB')
        if self.path:
            return ChromaEmbedder(
                self.model, path=self.path)
        elif self.host and self.port:
            return ChromaEmbedder(
                self.model, host=self.host, port=str(self.port))
        elif self.host:
            return ChromaEmbedder(
                self.model, host=self.host)
        else:
            return ChromaEmbedder(
                self.model)

    def embed_mixed(self, j: str, media: list=None) -> None:
        try:
            self.get_logger().info("embedding data")
            self.embed(j, media)
            self.get_logger().debug(
                f'embed_mixed: {j}')
        except Exception as e: 
            self.get_logger().error(
                f'embed_mixed: {traceback.format_exc()} Message: {j}')

    def embed_callback(self, msg: String) -> None:
        """Embeds the incoming JSON topic message into the Vector DB.
        Prints the incoming message also to the debug log.
        If it fails an error log entry will be written.
        """
        self.embed_mixed(msg)

    def embed_raw_callback(self, msg: String) -> None:
        """Embeds raw data of any type into default collection 'embed_raw'.
        To use another collection set the env var or ROS parameter accordingly.
        If the collection comes with the input data that will be used instead.
        """
        embed_msg = dict()
        try:
            data = json.loads(msg.data)

            if  'collection' in data \
                and 'documents' in data \
                and 'metadatas' in data:
                self.embed_callback(msg)

            elif isinstance(data, dict):
                embed_msg['collection'] = \
                    self.get_parameter('default_collection').value
                embed_msg['documents'] = (data['documents'] 
                    if 'documents' in data else [(data['data']
                        if 'data' in data else json.dumps(data))])
                embed_msg['metadatas'] = [data]
                if 'ids' in data:
                    embed_msg['ids'] = data['ids'] 
                self.embed_callback(
                    String(data=json.dumps(embed_msg)))

            else: raise Exception('none')

        except:
            embed_msg['collection'] = \
                self.get_parameter('default_collection').value
            embed_msg['documents'] = [msg.data]
            embed_msg['metadatas'] = [{'data': msg.data}]
            self.embed_callback(json.dumps(embed_msg))

    def embed_image(self, msg: Union[Image, TTImage]) -> None:
        try:
            from cv_bridge import CvBridge
            import cv2, PIL
            mat = CvBridge().imgmsg_to_cv2(msg 
                if isinstance(msg, Image) 
                else msg.image)
            mat = cv2.cvtColor(mat, cv2.COLOR_BGR2RGB)
            pil_img = PIL.Image.fromarray(mat)
            embed_msg = dict()
            embed_msg['collection'] = \
                self.get_parameter('default_collection').value

            if isinstance(msg, Image):
                ts ="%d.%09d" \
                    % (msg.header.stamp.sec, msg.header.stamp.nanosec)
                data = {
                    "metadata": [
                        {"key": "stamp", "value": float(ts)},
                        {"key": "frame_id", "value": msg.header.frame_id},
                        {"key": "tags", "value": ["fastembed", msg.header.frame_id]},
                        {"key": "type", "value": "image"}
                    ],
                    "data": ts
                }
                embed_msg['documents'] = [ts]
            else: 
                data = {'data': msg.json}
                embed_msg['documents'] = [msg.caption]

            embed_msg['metadatas'] = [data]
            self.embed_mixed(json.dumps(embed_msg), [pil_img])
        except Exception as e: 
            self.get_logger().error(
                f'embed_image: Exception: {traceback.format_exc()}')

    def embed_image_callback(self, msg: Image) -> None:
        """Embeds Image only with simple payload into default collection 'embed_raw'.
        To use another collection set the env var or ROS parameter accordingly.
        """
        self.embed_image(msg)

    def embed_ttimage_callback(self, msg: TTImage) -> None:
        """Embeds TTImage with payload into default collection 'embed_raw'.
        To use another collection set the env var or ROS parameter accordingly.
        """
        self.embed_image(msg)

def main():
    rclpy.init(args=None)
    n = EmbedderNode()
    rclpy.spin(n)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
