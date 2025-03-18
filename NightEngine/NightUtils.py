# NightUtils.py

from OpenGL.GL import *
import numpy as np

class NightUtils:

    @staticmethod
    def create_shader(shader_code, shader_type):
        """creates shader program from code. returns shader reference."""

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
        """creates and links program from shader code. returns program reference."""

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

    @staticmethod
    def create_vao():
        """creates vbo, binds it and returns its reference."""
        vao = glGenVertexArrays(1)
        glBindVertexArray(vao)
        return vao

    @staticmethod
    def create_vbo(data):
        """creates vbo, binds it, sends data to it and returns reference. ."""
        vbo = glGenBuffers(1)
        glBindBuffer(GL_ARRAY_BUFFER, vbo)
        glBufferData(GL_ARRAY_BUFFER, np.array(data, dtype=np.float32).ravel(), GL_STATIC_DRAW)
        return vbo

    @staticmethod
    def set_attribute_pointer(program, buffer, variable_name, data_type, stride=0, offset=None):
        """sets the attribute pointer for variable in shader program."""

        # --------------- bind vbo --------------- #
        
        glBindBuffer(GL_ARRAY_BUFFER, buffer)

        # ------- locate attrib in shader ------- #
        
        variable_reference = glGetAttribLocation(program, variable_name)

        if variable_reference == -1:
            print(f"Warning: Attribute {variable_name} not found in program {program}")
            return

        # --------- tell gpu how to read --------- #
        
        if data_type == "float":
            glVertexAttribPointer(variable_reference, 1, GL_FLOAT, False, stride, ctypes.c_void_p(offset))
        elif data_type == "vec2":
            glVertexAttribPointer(variable_reference, 2, GL_FLOAT, False, stride, ctypes.c_void_p(offset))
        elif data_type == "vec3":
            glVertexAttribPointer(variable_reference, 3, GL_FLOAT, False, stride, ctypes.c_void_p(offset))
        elif data_type == "vec4":
            glVertexAttribPointer(variable_reference, 4, GL_FLOAT, False, stride, ctypes.c_void_p(offset))
        else:
            raise Exception(f"Warning: Wrong attribute type: {data_type}.")

        # ------------ enable attrib ------------ #
        
        glEnableVertexAttribArray(variable_reference)

    @staticmethod
    def set_uniform(program, variable_name, data_type, data):

        # ------------- find uniform ------------- #

        glUseProgram(program)
        
        variable_reference = glGetUniformLocation(program, variable_name)

        if variable_reference == -1:
            print(f"Warning: Uniform {variable_name} not found in program {program}.")
            return

        # --------------- set data --------------- #

        if data_type == "int":
            glUniform1i(variable_reference, data)
        elif data_type == "bool":
            glUniform1i(variable_reference, data)
        elif data_type == "float":
            glUniform1f(variable_reference, data)
        elif data_type == "vec2":
            glUniform2f(variable_reference, data[0], data[1])
        elif data_type == "vec3":
            glUniform3f(variable_reference, data[0], data[1], data[2])
        elif data_type == "vec4":
            glUniform4f(variable_reference, data[0], data[1], data[2], data[3])
        elif data_type == "mat4":
            glUniformMatrix4fv(variable_reference, 1, GL_TRUE, data)
        else:
            raise Exception(f"Warning: Wrong uniform type {data_type}.")
