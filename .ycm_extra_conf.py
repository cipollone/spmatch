def Settings( **kwargs):
    includes = ['-I/home/roberto/Desktop/StereoPatch/SPMatch/include',
            '-isystem', '/usr/include/eigen3' ]
    flags = ['-x', 'c++', '-Wall', '-Wextra', '-O0', '-g', '-std=gnu++11']
    return {
            'flags': flags + includes
    }
