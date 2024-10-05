import csv
import os
from typing import List, Any

class CSVHandler:
    def __init__(self, folder_path: str = "", filename: str = "data.csv", header: List[str] = {}) -> None:
        
        self.folder_path = folder_path
        self.filename = filename

        self.header = header
        
        self._create_csv()

    @property
    def filename(self) -> str:
        return self._filename

    @filename.setter
    def filename(self, new_filename: str) -> None:
        if new_filename.find(".csv") == -1:
            raise ValueError("Filename is not a csv. Make sure to add the extension '.csv' to the file")
        self._filename = new_filename

    
    def _create_csv(self) -> None:
        if not os.path.exists(f'{self.folder_path}{self.filename}'):
            self._write_row(self.header)

    def _validate_data(self, data: List[Any]) -> bool:
        if len(data) != len(self.header):
            return False
        return True
    
    def _write_row(self, data: List[Any]) -> None:
        with open(f'{self.folder_path}{self.filename}', mode='a', newline='', encoding="utf-8") as file:
                writer = csv.writer(file)
                writer.writerow(data)
    
    def add_line(self, data: List[Any]) -> None:
        if self._validate_data(data):
            self._write_row(data)

