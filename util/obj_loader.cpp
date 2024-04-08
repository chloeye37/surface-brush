 #include "obj_loader.h"
#include <cstdio>
#include <iostream>

/**
 * @brief objLoader::loadFromFile
 * @param fileName
 * @return
 * Source: http://www.opengl-tutorial.org/beginners-tutorials/tutorial-7-model-loading/#example-obj-file
 */
pair<vector<Vector3f>, vector<Vector2i>> objLoader::loadFromFile(string fileName) {
    vector<Vector3f> vertices;
    vector<Vector2i> lineSegments;

    FILE * file = fopen(fileName.c_str(), "r");
    if( file == NULL ){
        cerr << "Impossible to open OBJ file !\n" << endl;
    }

    while( 1 ){

        char lineHeader[128]; // assume 1st word of line < 128 chars :)
        // read the first word of the line
        int res = fscanf(file, "%s", lineHeader);
        if (res == EOF)
            break; // EOF = End Of File. Quit the loop.

        // else : parse lineHeader
        if ( strcmp( lineHeader, "v" ) == 0 ){
            float vertex[3];
            fscanf(file, "%f %f %f\n", &vertex[0], &vertex[1], &vertex[2] );
            vertices.push_back(Vector3f(vertex[0], vertex[1], vertex[2]));
        } else if ( strcmp( lineHeader, "l" ) == 0 ){
            unsigned int vertexIndex[2];
            int matches = fscanf(file, "%d %d\n", &vertexIndex[0], &vertexIndex[1]);
            if (matches != 2){
                cerr << "File can't be read by our simple parser : ( Try exporting with other options\n" << endl;
            }
            lineSegments.push_back(Vector2i(vertexIndex[0]-1, vertexIndex[1]-1)); // offset of 1 to make the vertices 0-indexed instead of 1-indexed
        }
    }

    return make_pair(vertices, lineSegments);
}
