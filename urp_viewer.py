import gzip
import xml.etree.ElementTree as ET


def read_urp_file(file_path):
    with gzip.open(file_path, 'rb') as file:
        file_content = file.read()
    return file_content


def parse_urp_content(content):
    root = ET.fromstring(content)
    ET.dump(root)  # This will print the XML structure to the console


if __name__ == "__main__":
    file_path = "Force to distance.urp"
    content = read_urp_file(file_path)
    parse_urp_content(content.decode('utf-8'))  # Ensure the content is decoded to a string
