import os
from datetime import datetime
import logging
import re


def today_folder() -> str:
    date_now = datetime.now()
    log_path = f"/home/ego/Downloads/TCP_sensor/{date_now.year:04d}{date_now.month:02d}{date_now.day:02d}"
    if not os.path.exists(log_path):
        logging.info(f"created log path {log_path}")
        os.makedirs(log_path, exist_ok=True)
    else:
        logging.info(f"log path {log_path} exists.")
    return log_path