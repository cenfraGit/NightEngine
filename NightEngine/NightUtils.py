# NightUtils.py

from OpenGL.GL import *

class NightUtils:

    @staticmethod
    def create_shader(shader_code, shader_type):

        # ------ create and compile shader ------ #
        
        shader = glCreateShader(shader_type)
        glShaderSource(shader, shader_code)
        glCompileShader(shader)

        # ----------- check if success ----------- #

        if not glGetShaderiv(shader, GL_COMPILE_STATUS):
            glDeleteShader(shader)
            raise Exception('\n' + glGetShaderInfoLog(shader).decode("utf-8"))

        return shader

    @staticmethod
    def create_program(vertex_shader_code, fragment_shader_code):

        # ------------ create shaders ------------ #
        
        shader_vertex = NightUtils.create_shader(vertex_shader_code, GL_VERTEX_SHADER)
        shader_fragment = NightUtils.create_shader(fragment_shader_code, GL_FRAGMENT_SHADER)

        # ------------ create program ------------ #

        program = glCreateProgram()

        glAttachShader(program, shader_vertex)
        glAttachShader(program, shader_fragment)
        glLinkProgram(program)

        # ----------- check if success ----------- #

        if not glGetProgramiv(program, GL_LINK_STATUS):
            glDeleteProgram(program)
            raise Exception('\n' + glGetProgramInfoLog(program).decode("utf-8"))

        return program
