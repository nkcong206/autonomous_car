import redis
import os

HOST = os.environ.get("REDIS_HOST", "localhost")
PORT = os.environ.get("REDIS_PORT", 6379)
REDIS_PASSWORD = os.environ.get("REDIS_PASSWORD", "eYVX7EwVmmxKPCDmwMtyKVge8oLd2t81abc")

REDIT = redis.Redis(host=HOST, port=PORT, db=0, password=REDIS_PASSWORD)