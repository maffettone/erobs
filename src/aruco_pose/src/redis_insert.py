"""Copyright 2023 Brookhaven National Laboratory BSD 3 Clause License. See LICENSE.txt for details."""

import redis

# Connect to Redis
# Note: Change the host IP to map your server.
client = redis.Redis(host="192.168.56.1", port=6379, db=0)

# Step 1: Store tag data in Redis
client.hset("tag:1", mapping={"id": 0, "family": "DICT_APRILTAG_36h11", "size": 0.02665, "sample_name": "sample_1"})

client.hset("tag:2", mapping={"id": 150, "family": "DICT_APRILTAG_36h12", "size": 0.03000, "sample_name": "sample_2"})

# Step 2: Indexing the sample_name to the tag key (e.g., tag:1, tag:2)
client.hset("sample_name_index", "sample_1", "tag:1")
client.hset("sample_name_index", "sample_2", "tag:2")
