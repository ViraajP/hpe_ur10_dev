import gzip

with gzip.open("Lotus-Cut1.urp", "rb") as gz_file:
    with open("output.txt", "w") as txt_file:
        for line in gz_file:
            decoded_line = line.decode("utf-8")  # Decode bytes to string
            txt_file.write(decoded_line)