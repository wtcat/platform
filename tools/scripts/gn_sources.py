#!/usr/bin/python

import os
import sys

def generate_sources(dir, *k):
    with open("BUILD.gn.src", "w", encoding="utf-8") as f:
        f.write("component(xxxx) {\n"
                "  sources = [\n"
        )
        for root, dirs, files in os.walk(dir, topdown=True):
            for file in files:
                ext = os.path.splitext(file)[1]
                if ext != '':
                    if ext in k:
                        path = os.path.join(root, file)
                        if path[0] == '.':
                            path = path[2:]
                        f.write("    \"" + path + "\"" + ",\n")
        f.write("  ]\n"
                "}\n"
        )



if __name__ == "__main__":
    generate_sources(sys.argv[1], ".c", ".cc", "cpp")
