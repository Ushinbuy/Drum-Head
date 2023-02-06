import re
import sys

def convertWavToHex(fileInp):
    with open(fileInp, 'rb') as file_to_read:
        content = file_to_read.read()

    newFileName = fileInp.replace('wav','txt')
    newContent = ','.join(map('0x{:02x}'.format, content))

    newContent = re.sub("(.{80})", "\\1\n", newContent, 0 , re.DOTALL)

    print(newContent[:200])

    file_to_write = open(newFileName, 'w', encoding="utf-8")
    file_to_write.write(newContent)
    file_to_write.close()


if __name__ == "__main__":
    fileName = sys.argv[1]
    convertWavToHex(fileName)