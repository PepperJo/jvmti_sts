#! /usr/bin/python

import sys
from pprint import pprint


def parseFile(fname):
    fin = open(fname,'r')
    return eval(fin.read())


def help():
    print 'Usage: python parse.py <filenames>'


if __name__ == '__main__':
    import sys    
    try:
        fnames = sys.argv[1:]
    except: 
        help()
        exit()    
    for f in fnames:
        samples = parseFile(f)
        pprint(samples)

