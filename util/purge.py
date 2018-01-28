#!/usr/bin/python
import sys
import getopt
import os


def main(argv):
    input_file = ''
    output_file = "tmp.osm"
    try:
        opts, args = getopt.getopt(argv, "hi:o:", ["ifile="])
    except getopt.GetoptError:
        print 'purge.py -i <input_file>'
        sys.exit(2)
    for opt, arg in opts:
        if opt == '-h':
            print 'purge.py -i <input_file>'
            sys.exit()
        elif opt in ("-i", "--ifile"):
            input_file = arg.strip()
    print 'Input file is "', input_file

    purge(input_file, output_file)


def purge(input_file, output_file):
    delete = False
    open_tag = ""
    with open(input_file) as f:
        with open(output_file, 'w') as wf:
            for line in f:
                if "action='delete'" in line:
                    delete = "/>" not in line
                    open_tag = get_open_tag(line)
                    continue

                if not delete:
                    wf.write(line)

                if "</" + open_tag + ">" in line:
                    delete = False
        wf.close()
    f.close()
    os.remove(input_file)
    os.rename(output_file, input_file)


def get_open_tag(line):
    open_char_i = line.index("<")
    return line[open_char_i + 1:open_char_i + line[open_char_i:].index(" ")]

if __name__ == "__main__":
    main(sys.argv[1:])
    print 'Done.'
