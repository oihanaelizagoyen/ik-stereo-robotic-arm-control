import csv
import os
from datetime import datetime

class CSVLogger:
    def __init__(self, fieldnames):
        self.field_names = fieldnames
        self.log_directory = os.path.join(os.path.dirname(__file__), "logs")
        os.makedirs(self.log_directory, exist_ok=True)

    def get_log_file_path(self):
        timestamp = datetime.now().strftime(f"%Y%m%d")
        return os.path.join(self.log_directory, f"log_{timestamp}.csv")

    def log(self, data: dict):
        file_path = self.get_log_file_path()
        file_exists = os.path.exists(file_path)

        with open(file_path, mode='a', newline='') as csvfile:
            writer = csv.DictWriter(csvfile, fieldnames=self.field_names)
            if not file_exists:
                writer.writeheader()
            writer.writerow(data)
