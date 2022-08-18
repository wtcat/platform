#!/usr/bin/python

#
# Install Python module:
#   pip install robotpy-cppheaderparser
#
# Copy to user binary directory
#   sudo cp -v cpp_parser.py /usr/local/bin
#
# Example: 
#   cpp_parser.py xxx.h
# 

import CppHeaderParser
import subprocess
import sys
import os

def clang_syntax_check(incfile):
    cmdList = ['clang', '-Xclang', '-fsyntax-only']
    cmdList.append(incfile)
    return subprocess.call(cmdList, shell=False)

def generate_file_header(fp, incfile):
    str = """
/*
 * Copyright 
 */
    """
    str += "\n#include \"{}\"\n".format(incfile)
    fp.write(str)

def generate_file_foot(fp):
    return

def generate_instance_header(fp, className):
    str = className + ' __xxxx_instance = {\n'
    fp.write(str)

def generate_instance_foot(fp):
    fp.write('};\n')

def generate_definition(name, type):
    expr = type.replace('( * )', name)
    expr += ' {\n\treturn_result \n}\n\n'
    if expr[0:4] == 'void':
        expr = expr.replace('return_result', '')
    else:
        expr = expr.replace('return_result', 'return 0;')
    ret = 'static ' + expr
    return ret

def generate_class_methods(className, cppHeader):
    funcList = []
    fieldList = []
    publicField = cppHeader.classes[className]['properties']['public']
    for method in publicField:
        type_txt = method['type']
        if type_txt.count('(') == 2:
            funcList.append(generate_definition(method['name'], type_txt))
            classField = '\t.' + method['name'] + ' = ' + method['name'] + ',\n'
            fieldList.append(classField)
    return funcList, fieldList

def generate_source(fileName):
    cppHeader = CppHeaderParser.CppHeader(fileName)
    #Get source file name
    extName = os.path.splitext(fileName)
    srcName = extName[0] + '.c'

    #Flush to file
    with open(srcName, 'w') as f:
        generate_file_header(f, fileName)
        funclist = []
        fieldList = []

        for className in cppHeader.classes.keys():
            funclist, fieldList = generate_class_methods(className, cppHeader)
            if funclist:
                f.write('\n')
                for content in funclist:
                    f.write(content)
                generate_instance_header(f, className)
                for fieldText in fieldList:
                    f.write(fieldText)
                generate_instance_foot(f)
        generate_file_foot(f)
    print('Generate file: ', srcName)

def main():
    if len(sys.argv) != 2:
        print("Usage: cpp_parse.py [header file name]")
        sys.exit()
    fileName = sys.argv[1]
    if os.path.exists(fileName):
        if clang_syntax_check(fileName) == 0:
            generate_source(fileName)

if __name__ == '__main__':
    main()