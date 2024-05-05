#include "mesh.h"

#include <iostream>
#include <fstream>
#include <array>


#include <QFileInfo>
#include <QString>
#include <Eigen>

#define TINYOBJLOADER_IMPLEMENTATION
#include "util/tiny_obj_loader.h"
#include "util/ply_loader.h"
#include "util/obj_loader.h"

#include <fstream>

// -------- PUBLIC STARTS -------------------------------------------------------------------------------
Mesh::Mesh()
{
    this->settings = Settings::getInstance();
}

void Mesh::loadFromFile()
{
    // load obj file
    pair<vector<Vector3f>, vector<Vector2i>> verticesAndLineSegments = objLoader::loadFromFile(this->settings->inObjFile);
    vector<Vector3f> vertices = verticesAndLineSegments.first;
    vector<Vector2i> lineSegments = verticesAndLineSegments.second;
    vector<vector<int>> lines = this->parseToPolyline(lineSegments);

    // load ply file
    vector<pair<Vector3f,float>> normalsAndStrokeWidths = plyLoader::loadFromFile(this->settings->inPlyFile);

    // populate the _vertices vector
    vector<Vertex *> m_vertices;
    for (int i = 0; i < vertices.size(); i++)
    {
        Vertex *vertex = new Vertex(vertices[i], true, normalsAndStrokeWidths[i].first, normalsAndStrokeWidths[i].second);
        // vertex.tangent is set later in calculateTangents()
        m_vertices.push_back(vertex);
    }

    this->_lines = lines;
    this->_vertices = m_vertices;

    std::vector<Vector3f> normals = std::vector<Vector3f>();
    for(int i = 0; i < normalsAndStrokeWidths.size(); i++){
        normals.push_back(normalsAndStrokeWidths[i].first);
    }

    calculateTangentsAndBinormals(vertices, normals);
}

void Mesh::saveToFile()
{
    ofstream outStrokeFile;
    outStrokeFile.open(this->settings->outStrokeFile);

    ofstream outMeshFile;
    outMeshFile.open(this->settings->outMeshFile);

    // Write vertices
    for (size_t i = 0; i < _vertices.size(); i++)
    {
        Vertex *v = _vertices[i];
        outMeshFile << "v " << v->position[0] << " " << v->position[1] << " " << v->position[2] << endl;
        outStrokeFile << "v " << v->position[0] << " " << v->position[1] << " " << v->position[2] << endl;
    }

    // Write vertex normals
    for (size_t i = 0; i < _vertices.size(); i++)
    {
        Vertex *v = _vertices[i];
        outMeshFile << "vn " << v->normal[0] << " " << v->normal[1] << " " << v->normal[2] << endl;
    }

    // Write faces (MESH ONLY)
    for (size_t i = 0; i < _faces.size(); i++)
    {
        const Vector3i &f = _faces[i];
        outMeshFile << "f " << (f[0] + 1) << " " << (f[1] + 1) << " " << (f[2] + 1) << endl;
    }

    // Write line segments (STROKE ONLY)
    for (size_t i = 0; i < _lines.size(); i++)
    {
        const vector<int> &l = _lines[i];
        for (size_t j = 0; j < l.size() - 1; j++)
        {
            outStrokeFile << "l " << (l[j] + 1) << " " << (l[j + 1] + 1) << endl;
        }
    }

    outStrokeFile.close();
    outMeshFile.close();
}

Vector3i sortvec(Vector3i vec){
    int a = vec[0];
    int b = vec[1];
    int c = vec[2];
    if(a<=b && a<=c){
        if(b < c){
            return Vector3i(a,b,c);
        }
        else{
            return Vector3i(a,c,b);
        }
    }
    else if(b <= a && b <= c){
        if(a < c){
            return Vector3i(b,a,c);
        }
        else{
            return Vector3i(b,c,a);
        }
    }
    else{
        if(a < b){
            return Vector3i(c,a,b);
        }
        else{
            return Vector3i(c,b,a);
        }
    }
}

int tohash(Vector3i k, int N){
    Vector3i sortedvec = sortvec(k);
    return sortedvec[0]*N*N + sortedvec[1]*N + sortedvec[2];
}

int dig(int x, int i, int N){
    return ((x%static_cast<int>(pow(N,i+1)))-(x%static_cast<int>(pow(N,i))))/pow(N,i);
}

Vector3i fromhash(int x, int N){
    return Vector3i(dig(x,2,N),  dig(x,1,N), dig(x,0,N));
}

void Mesh::debugSaveToFile()
{
    ofstream outStrokeFile;
    outStrokeFile.open(this->settings->outStrokeFile);
    ofstream outMeshFile;
    outMeshFile.open(this->settings->outMeshFile);

    // Write vertices
    for (size_t i = 0; i < _vertices.size(); i++)
    {
        Vertex *v = _vertices[i];
        outStrokeFile << "v " << v->position[0] << " " << v->position[1] << " " << v->position[2] << endl;
        outMeshFile << "v " << v->position[0] << " " << v->position[1] << " " << v->position[2] << endl;
    }

    // Write strokes
    // Write original line segments
    for (size_t i = 0; i < _lines.size(); i++)
    {
        const vector<int> &l = _lines[i];
        for (size_t j = 0; j < l.size() - 1; j++)
        {
            outStrokeFile << "l " << (l[j] + 1) << " " << (l[j + 1] + 1) << endl;
            outMeshFile << "l " << (l[j] + 1) << " " << (l[j + 1] + 1) << endl;
        }
    }
    // Write a line segment if there is a match between two points
    for (size_t i = 0; i < _vertices.size(); i++)
    {
        if (leftMatch.contains(i))
        {
            if (leftMatch.at(i) != -1)
            {
                outStrokeFile << "l " << i + 1 << " " << leftMatch.at(i) + 1 << endl;
                outMeshFile << "l " << i + 1 << " " << leftMatch.at(i) + 1 << endl;
            }
        }
        if (rightMatch.contains(i))
        {
            if (rightMatch.at(i) != -1)
            {
                outStrokeFile << "l " << i + 1 << " " << rightMatch.at(i) + 1 << endl;
                outMeshFile << "l " << i + 1 << " " << rightMatch.at(i) + 1 << endl;
            }
        }
    }

    //Look at all face hashes
    std::vector<int> facehashes = std::vector<int>();
    for(int i = 0; i < _faces.size(); i++){
        if((_faces[i][0]==_faces[i][1]) || (_faces[i][1]==_faces[i][2]) || (_faces[i][2]==_faces[i][0])){
            // std::cout << "Duplicate vertices! " + std::to_string(_faces[i][0]) + ", " + std::to_string(_faces[i][1]) + ", " + std::to_string(_faces[i][2]) << std::endl;
        }
    }

    // Write faces (MESH ONLY)
    for (size_t i = 0; i < _faces.size(); i++)
    {
        const Vector3i &f = _faces[i];
        outMeshFile << "f " << (f[0] + 1) << " " << (f[1] + 1) << " " << (f[2] + 1) << endl;
    }

    outStrokeFile.close();
    outMeshFile.close();
}


void Mesh::debugUndecidedTrianglesSaveToFile()
{
    ofstream outStrokeFile;
    outStrokeFile.open(this->settings->outStrokeFile);
    ofstream outMeshFile;
    outMeshFile.open(this->settings->outMeshFile);

    // we have: vector<vector<Vector3i>> undecidedTriangles;
    set<int> undecided_vertices;
    vector<int> new_vertex_list;
    vector<Vector3i> undecided_faces;
    unordered_map<int, int> old_to_new; // map of vertex indices from old (in _vertices) to new (in undecided_vertices)
    unordered_map<int, int> new_to_old;
    for (auto & triangle_list : undecidedTriangles) {
        for (auto triangle : triangle_list) {
            undecided_vertices.insert(triangle[0]);
            undecided_vertices.insert(triangle[1]);
            undecided_vertices.insert(triangle[2]);
        }
    }
    int new_index = 0;
    for (auto vertex : undecided_vertices) {
        new_vertex_list.push_back(new_index);
        old_to_new.insert({vertex, new_index});
        new_to_old.insert({new_index, vertex});
        new_index++;
    }
    for (auto new_vertex : new_vertex_list) {
        outMeshFile << "v " << _vertices[new_to_old.at(new_vertex)]->position[0] << " " << _vertices[new_to_old.at(new_vertex)]->position[1] << " " << _vertices[new_to_old.at(new_vertex)]->position[2] << endl;
        outStrokeFile << "v " << _vertices[new_to_old.at(new_vertex)]->position[0] << " " << _vertices[new_to_old.at(new_vertex)]->position[1] << " " << _vertices[new_to_old.at(new_vertex)]->position[2] << endl;
    }


    for (auto & triangle_list : undecidedTriangles) {
        for (auto triangle : triangle_list) {

            // then write lines
            outMeshFile << "l " << (old_to_new.at(triangle[0]) + 1) << " " << (old_to_new.at(triangle[1]) + 1) << endl;
            outMeshFile << "l " << (old_to_new.at(triangle[1]) + 1) << " " << (old_to_new.at(triangle[2]) + 1) << endl;
            outMeshFile << "l " << (old_to_new.at(triangle[2]) + 1) << " " << (old_to_new.at(triangle[0]) + 1) << endl;
            // then write faces for mesh file
            outMeshFile << "f " << (old_to_new.at(triangle[0]) + 1) << " " << (old_to_new.at(triangle[1]) + 1) << " " << (old_to_new.at(triangle[2]) + 1) << endl;

            outStrokeFile << "l " << (old_to_new.at(triangle[0]) + 1) << " " << (old_to_new.at(triangle[1]) + 1) << endl;
            outStrokeFile << "l " << (old_to_new.at(triangle[1]) + 1) << " " << (old_to_new.at(triangle[2]) + 1) << endl;
            outStrokeFile << "l " << (old_to_new.at(triangle[2]) + 1) << " " << (old_to_new.at(triangle[0]) + 1) << endl;
        }
    }

    outStrokeFile.close();
    outMeshFile.close();
}
// Preprocess the lines: check if the strokes have an abrupt direction change (angle of 45◦ or less between consecutive tangents)
// within 15% of overall stroke length from either end and remove the offending end-sections.
void Mesh::preprocessLines()
{
    vector<vector<int>> new_lines;
    int itr = 0;
    // process each line at a time
    for (auto &line : _lines)
    {
        // first calculate line length -- sum of all segment lengths
        float total_length = 0;
        for (int i = 1; i < line.size(); i++)
        {
            // distance between vertex i and vertex i-1
            total_length += (this->_vertices[line[i]]->position - this->_vertices[line[i - 1]]->position).norm();
        }

        float length_covered = 0;
        // forward direction: start to end until length_covered > 0.15 * total_length
        // calculate the tangents along the way and test if they differ by more than 45 degrees
        Vector3f prev_tangent = this->_vertices[line[1]]->position - this->_vertices[line[0]]->position; // tangent of the first vertex is just the line segment direction
        int cur_vert = 0;
        int next_vert = 1;
        int cut_pos_forward = 0; // the vertex at which we should cut the line (remove every vertex before this cut_pos)
        while (length_covered < 0.15 * total_length)
        {
            Vector3f curA = this->_vertices[line[cur_vert]]->position;
            Vector3f curB = this->_vertices[line[next_vert]]->position;
            Vector3f curC = this->_vertices[line[next_vert + 1]]->position;
            Vector3f AC = curC - curA;
            Vector3f B_normal = this->_vertices[line[next_vert]]->normal;
            Vector3f AC_parallel = AC.dot(B_normal) / (B_normal.norm() * B_normal.norm()) * B_normal; // AC projected onto the direction of normal

            Vector3f cur_tangent = (AC - AC_parallel).normalized(); // tangent at B
            if (acos(cur_tangent.dot(prev_tangent) / (cur_tangent.norm() * prev_tangent.norm())) > M_PI / 4.0)
            {
                // remove everything before vertex B (starting from A)
                cut_pos_forward = next_vert;
            }
            // move on to the next segment
            prev_tangent = cur_tangent;
            length_covered += (curB - curA).norm();
            cur_vert++;
            next_vert++;
        }

        // backward direction: end to start
        int n = line.size();
        length_covered = 0;
        prev_tangent = this->_vertices[line[n - 2]]->position - this->_vertices[line[n - 1]]->position; // tangent of the first vertex is just the line segment direction
        cur_vert = n - 1;
        next_vert = n - 2;
        int cut_pos_backward = n - 1; // the vertex at which we should cut the line (remove every vertex after this cut_pos)
        while (length_covered < 0.15 * total_length)
        {
            Vector3f curA = this->_vertices[line[cur_vert]]->position;
            Vector3f curB = this->_vertices[line[next_vert]]->position;
            Vector3f curC = this->_vertices[line[next_vert - 1]]->position;
            Vector3f AC = curC - curA;
            Vector3f B_normal = this->_vertices[line[next_vert]]->normal;
            Vector3f AC_parallel = AC.dot(B_normal) / (B_normal.norm() * B_normal.norm()) * B_normal; // AC projected onto the direction of normal

            Vector3f cur_tangent = (AC - AC_parallel).normalized(); // tangent at B
            if (acos(cur_tangent.dot(prev_tangent) / (cur_tangent.norm() * prev_tangent.norm())) > M_PI / 4.0)
            {
                // remove everything before vertex B (starting from A)
                cut_pos_backward = next_vert;
            }
            // move on to the next segment
            prev_tangent = cur_tangent;
            length_covered += (curB - curA).norm();
            cur_vert--;
            next_vert--;
        }

        // remove everything before cut_pos_forward and after cut_pos_backward
        // need to modify: _vertices (vector<Vector3f>), _lines (vector<vector<int>>), _vertexNormals (vector<Vector3f>)
        vector<int> new_line;
        for (int i = cut_pos_forward; i <= cut_pos_backward; i++)
        {
            new_line.push_back(line[i]);
        }
        new_lines.push_back(new_line);

        // logical removal of vertices
        for (int i = 0; i < cut_pos_forward; i++)
        {
            _vertices[line[i]]->isActive = false;
        }

        for (int i = n - 1; i > cut_pos_backward; i--)
        {
            _vertices[line[i]]->isActive = false;
        }
        itr++;
    }
    _lines = new_lines;

    // update the _vertices and _lines (using the correct indices)
    // - update _vertices
    unordered_map<int, int> index_map; // old index -> new index
    int active_number = 0;
    vector<Vertex *> new_vertices;
    for (int i = 0; i < _vertices.size(); i++)
    {
        if (_vertices[i]->isActive)
        {
            index_map[i] = active_number;
            active_number++;
            new_vertices.push_back(this->_vertices[i]);
        }
        else
        {
            delete this->_vertices[i];
        }
    }
    this->_vertices = new_vertices;

    // - remap the vertex indices in _lines
    vector<vector<int>> final_lines;
    for (auto &line : _lines)
    {
        vector<int> final_line;
        for (int vertex : line)
        {
            final_line.push_back(index_map[vertex]);
        }
        final_lines.push_back(final_line);
    }
    this->_lines = final_lines;

    //NEW NEW NEW NEW
    for(int i = 0; i < this->_lines.size(); i++){
        for(int j = 0; j < (this->_lines[i]).size(); j++){
            vertsToStrokes.emplace(this->_lines[i][j], i);
        }
    }
}

void Mesh::getRestrictedMatchingCandidates()
{
    assert(!this->_lines.empty());

    for (int i = 0; i < this->_lines.size(); i++)
    {
        vector<int> stroke = this->_lines[i];
        // split rest of strokes into left & right strokes
        pair<vector<int>, vector<int>> split = this->splitStrokesIntoLeftRight(i);
        vector<int> leftStrokeIndices = split.first;
        vector<int> rightStrokeIndices = split.second;

        int highestLeftMatches = 0;
        int leftDominantStrokeIndex = -1;
        for (int leftStrokeIndex : leftStrokeIndices)
        {
            // get highest matches stroke
            int matches = this->calcNumberOfMatches(i, leftStrokeIndex, true);
            if (matches > highestLeftMatches)
            {
                leftDominantStrokeIndex = leftStrokeIndex;
                highestLeftMatches = matches;
            }
        }
        // if leftDominantStrokeIndex == -1, then no left dominant stroke!

        int highestRightMatches = 0;
        int rightDominantStrokeIndex = -1;
        for (int rightStrokeIndex : rightStrokeIndices)
        {
            // get highest matches stroke
            int matches = this->calcNumberOfMatches(i, rightStrokeIndex, false);
            if (matches > highestRightMatches)
            {
                rightDominantStrokeIndex = rightStrokeIndex;
                highestRightMatches = matches;
            }
        }
        // if rightDominantStrokeIndex == -1, then no right dominant stroke!

        // get restricted matching candidates
        for (int strokeVertexIndex = 0; strokeVertexIndex < stroke.size(); strokeVertexIndex++)
        {
            int vertexIndex = stroke[strokeVertexIndex]; // index of vertex in vector of vertices
            if (!this->validVertexVertexMatch.contains(vertexIndex))
            {
                this->validVertexVertexMatch[vertexIndex] = unordered_map<int, unordered_set<int>>();
            }
            unordered_map<int, unordered_set<int>> dominantStrokeMaps = this->validVertexVertexMatch.at(vertexIndex);

            unordered_set<int> vertexLeftRestrictedMatchingCandidates;
            if (leftDominantStrokeIndex != -1)
            {
                if (!dominantStrokeMaps.contains(leftDominantStrokeIndex))
                {
                    dominantStrokeMaps[leftDominantStrokeIndex] = unordered_set<int>();
                }
                unordered_set<int> leftDominantStrokeMatches = dominantStrokeMaps.at(leftDominantStrokeIndex);
                vertexLeftRestrictedMatchingCandidates.merge(leftDominantStrokeMatches);
            }
            this->leftRestrictedMatchingCandidates[vertexIndex] = vertexLeftRestrictedMatchingCandidates;

            unordered_set<int> vertexRightRestrictedMatchingCandidates;
            if (rightDominantStrokeIndex != -1)
            {
                if (!dominantStrokeMaps.contains(rightDominantStrokeIndex))
                {
                    dominantStrokeMaps[rightDominantStrokeIndex] = unordered_set<int>();
                }
                unordered_set<int> rightDominantStrokeMatches = dominantStrokeMaps.at(rightDominantStrokeIndex);
                vertexRightRestrictedMatchingCandidates.merge(rightDominantStrokeMatches);
            }
            this->rightRestrictedMatchingCandidates[vertexIndex] = vertexRightRestrictedMatchingCandidates;
        }

        // also consider vertices in the same stroke
        for (int strokeVertexIndex = 0; strokeVertexIndex < stroke.size(); strokeVertexIndex++)
        {
            int vertexIndex = stroke[strokeVertexIndex];
            for (int strokeVertexIndex2 = 0; strokeVertexIndex2 < stroke.size(); strokeVertexIndex2++)
            {
                int vertexIndex2 = stroke[strokeVertexIndex2];

                if (vertexIndex == vertexIndex2)
                    continue;

                bool isEitherVertexCloseToEnd = (vertexIndex > stroke.size() - this->settings->noOfNearEndVerticesToConsider || vertexIndex < this->settings->noOfNearEndVerticesToConsider || vertexIndex2 > stroke.size() - this->settings->noOfNearEndVerticesToConsider || vertexIndex2 < this->settings->noOfNearEndVerticesToConsider);

                if (this->doTwoVerticesMatch(vertexIndex, vertexIndex2, true, true, i, isEitherVertexCloseToEnd))
                {
                    if (!this->leftRestrictedMatchingCandidates.contains(vertexIndex))
                    {
                        this->leftRestrictedMatchingCandidates[vertexIndex] = unordered_set<int>();
                    }
                    unordered_set<int> leftMatches = this->leftRestrictedMatchingCandidates.at(vertexIndex);
                    leftMatches.insert(vertexIndex2);
                    this->leftRestrictedMatchingCandidates[vertexIndex] = leftMatches;
                }
                else if (this->doTwoVerticesMatch(vertexIndex, vertexIndex2, false, true, i, isEitherVertexCloseToEnd))
                {
                    if (!this->rightRestrictedMatchingCandidates.contains(vertexIndex))
                    {
                        this->rightRestrictedMatchingCandidates[vertexIndex] = unordered_set<int>();
                    }
                    unordered_set<int> rightMatches = this->rightRestrictedMatchingCandidates.at(vertexIndex);
                    rightMatches.insert(vertexIndex2);
                    this->rightRestrictedMatchingCandidates[vertexIndex] = rightMatches;
                }
            }
        }
    }
}

void Mesh::meshStripGeneration() {
    unordered_set<int> trihashes;

    for(int i = 0; i < _lines.size(); i++){
        //Go through the stroke
        std::vector<int> currstroke = _lines[i];
        for(int j = 0; j < currstroke.size() - 1; j++){
            //Go through each vertex
            int pi = currstroke[j];
            //Check if the next vertex even exists
            if(leftMatch.contains(pi) && leftMatch.contains(pi+1)){
                int qi = leftMatch.at(pi);
                int qn = leftMatch.at(pi+1);
                if((qi >= 0)&&(qn >= 0) && (vertsToStrokes.at(qi) == vertsToStrokes.at(qn))){
                    //Both this and the next vertex have matches.
                    std::vector<Vector3i> outtriangles = triangulatePair(pi,qi,pi+1,qn);
                    for(int k = 0; k < outtriangles.size(); k ++){
                        //Check if the triangle exists already
                        int trihash = tohash(outtriangles[k],_vertices.size());
                        if(!trihashes.contains(trihash)){
                            _faces.push_back(outtriangles[k]);
                            trihashes.insert(trihash);
                        }
                    }
                }
            }
            if(rightMatch.contains(pi) && rightMatch.contains(pi+1)){
                int qi = rightMatch.at(pi);
                int qn = rightMatch.at(pi+1);
                if((qi >= 0)&&(qn >= 0) && (vertsToStrokes.at(qi) == vertsToStrokes.at(qn))){
                    //Both this and the next vertex have matches.
                    std::vector<Vector3i> outtriangles = triangulatePair(pi,qi,pi+1,qn);
                    for(int k = 0; k < outtriangles.size(); k ++){
                        //Check if the triangle exists already
                        int trihash = tohash(outtriangles[k],_vertices.size());
                        if(!trihashes.contains(trihash)){
                            _faces.push_back(outtriangles[k]);
                            trihashes.insert(trihash);
                        }
                    }
                }
            }
        }
    }
}

void Mesh::cleanUp()
{
    for (int i = 0; i < this->_vertices.size(); i++)
    {
        delete this->_vertices[i];
    }
}

vector<vector<int>> Mesh::getLines()
{
    return _lines;
}

// -------- PUBLIC ENDS -------------------------------------------------------------------------------

// -------- PRIVATE STARTS -------------------------------------------------------------------------------

vector<vector<int>> Mesh::parseToPolyline(vector<Vector2i> connections)
{
    // TODO: Sort connections first by the first index!
    vector<Vector2i> sortedconns = connections;

    struct sort_pred
    {
        bool operator()(const Vector2i &left, const Vector2i &right)
        {
            return left[0] < right[0];
        }
    };

    std::sort(sortedconns.begin(), sortedconns.end(), sort_pred());

    std::vector<std::vector<int>> polylines = std::vector<std::vector<int>>();
    int index = 0;
    int strokedex = 0;
    while (index < sortedconns.size())
    {
        //Add vertices to the vertsToStrokes map!
        vector<int> currentpoly = std::vector<int>();
        currentpoly.push_back(sortedconns[index][0]);
        currentpoly.push_back(sortedconns[index][1]);

        index = index + 1;
        while (sortedconns[index][0] == currentpoly[currentpoly.size() - 1])
        {
            currentpoly.push_back(sortedconns[index][1]);
            index = index + 1;
        }
        polylines.push_back(currentpoly);
        strokedex = strokedex + 1;
    }

    return polylines;
}



// --------------------- Section 5.4 ------------------------
// --------------------- Get undecided triangle clusters ------------------------
// A function to create a map that associates each vertex with its list of triangles, and a map that associates every edge with its triangles
void Mesh::populateTriangleMaps() {
    for (Vector3i triangle : _faces) {
        // first populate unordered_map<std::pair<int, int>, Vector3i> edgeToTriangles
        int v1 = triangle[0]; int v2 = triangle[1]; int v3 = triangle[2];
        auto pair12 = v1 < v2 ? std::make_pair(v1,v2) : std::make_pair(v2,v1); // edge ordered by smaller vertex -- larger vertex
        if (edgeToTriangles.contains(pair12)) {
            edgeToTriangles.at(pair12).push_back(triangle);
        }else {
            edgeToTriangles.insert({pair12, {triangle}});
        }
        auto pair13 = v1 < v3 ? std::make_pair(v1,v3) : std::make_pair(v3,v1); // edge ordered by smaller vertex -- larger vertex
        if (edgeToTriangles.contains(pair13)) {
            edgeToTriangles.at(pair13).push_back(triangle);
        }else {
            edgeToTriangles.insert({pair13, {triangle}});
        }
        auto pair23 = v3 < v2 ? std::make_pair(v3,v2) : std::make_pair(v2,v3); // edge ordered by smaller vertex -- larger vertex
        if (edgeToTriangles.contains(pair23)) {
            edgeToTriangles.at(pair23).push_back(triangle);
        }else {
            edgeToTriangles.insert({pair23, {triangle}});
        }

        // then populate unordered_map<int, Vector3i> vertexToTriangles
        if (vertexToTriangles.contains(v1)) {
            vertexToTriangles.at(v1).push_back(triangle);
        }else {
            vertexToTriangles.insert({v1, {triangle}});
        }
        if (vertexToTriangles.contains(v2)) {
            vertexToTriangles.at(v2).push_back(triangle);
        }else {
            vertexToTriangles.insert({v2, {triangle}});
        }
        if (vertexToTriangles.contains(v2)) {
            vertexToTriangles.at(v2).push_back(triangle);
        }else {
            vertexToTriangles.insert({v2, {triangle}});
        }
    }
}

// We have all the triangles (triangles: vector<int> = {v1, v2, v3}), need to identify the incompatible ones
// Populates vector<vector<Vector3i>> undecidedTriangles
void Mesh::computeUndecidedTriangles() {
    populateTriangleMaps();
    vector<vector<Vector3i>> res;
    std::set<std::pair<int,int>> undecided_edges;
    // std::set<int> undecided_vertices;
    // Case (1) and (3): look for triangles that share one edge
    for (auto i = edgeToTriangles.begin(); i != edgeToTriangles.end(); i++) {
        auto cur_edge = i->first;
        auto [v1,v2] = i->first;
        if (abs(v1 - v2) > 1) continue; // only consider the case where shared edge is on the same stroke
        vector<Vector3i> triangles = i->second;
        // loop through all triangles and check them pairwise
        for (Vector3i t1 : triangles) {
            for (Vector3i t2 : triangles) {
                int v3, v4;
                // get v3 and v4
                if (t1[0] != v1 && t1[0] != v2) v3 = t1[0];
                else if (t1[1] != v1 && t1[1] != v2) v3 = t1[1];
                else v3 = t1[2];
                if (t2[0] != v1 && t2[0] != v2) v4 = t2[0];
                else if (t2[1] != v1 && t2[1] != v2) v4 = t2[1];
                else v4 = t2[2];
                if (v3 == v4) continue; // same triangle...
                // Case (1): check if v3 and v4 are on different sides of the stroke
                Vector3f edge = _vertices[v1]->position - _vertices[v2]->position;
                Vector3f t1edge = _vertices[v3]->position - _vertices[v2]->position;
                Vector3f t2edge = _vertices[v4]->position - _vertices[v2]->position;
                if ((t1edge.dot(_vertices[v2]->binormal)) * (t2edge.dot(_vertices[v2]->binormal)) > 0) { // same side
                    // mark this edge
                    undecided_edges.insert(cur_edge);
                    // std::cout << "Edge shared. v1: " << v1 << " | v2: " << v2 << " | v3: " << v3  << " | v4: " << v4 << std::endl;
                    // std::cout << "Edge shared: intersection" << std::endl;
                }
                // Case (3): check if the dihedral angle is less than 45 degrees
                // https://math.stackexchange.com/questions/47059/how-do-i-calculate-a-dihedral-angle-given-cartesian-coordinates
                Vector3f b2 = edge;
                Vector3f b1 = - t1edge;
                Vector3f b3 = t2edge;
                Vector3f n1 = b1.cross(b2).normalized();
                Vector3f n2 = b2.cross(b3).normalized();
                float x = n1.dot(n2);
                float y = (n1.cross(b2.normalized())).dot(n2);
                if (abs(atan2(y,x) * 180 / M_PI) < 45) { // less than 45 degrees
                    undecided_edges.insert(cur_edge);
                    // std::cout << "Edge shared: dihedral less than 45" << std::endl;
                }
            }
        }
    }
    // Case (2): look for triangles that share one vertex
    for (auto i = vertexToTriangles.begin(); i != vertexToTriangles.end(); i++) {
        int v = i->first;
        vector<Vector3i> triangles = i->second;
        // loop through all triangles and check them pairwise
        for (Vector3i t1 : triangles) {
            for (Vector3i t2 : triangles) {
                // first check if they are on the same side of the stroke
                // v1 and v2 belong to t1, v3 and v4 belong to t2
                int v1, v2, v3, v4;
                if (t1[0]==v) {
                    v1 = t1[1]; v2 = t1[2];
                }else if (t1[1]==v) {
                    v1 = t1[0]; v2 = t1[2];
                }else {
                    v1 = t1[0]; v2 = t1[1];
                }
                if (t2[0]==v) {
                    v3 = t2[1]; v4 = t2[2];
                }else if (t2[1]==v) {
                    v3 = t2[0]; v4 = t2[2];
                }else {
                    v3 = t2[0]; v4 = t2[1];
                }
                if ((v1==v3 && v2==v4) || (v1==v4 && v2==v3)) continue; // same triangle
                if (abs(v1-v2) > 1 || abs(v3-v4) > 1) continue; // non-consecutive edges for the opposite two edges
                if (((_vertices[v1]->position - _vertices[v]->position).dot(_vertices[v2]->binormal)) * ((_vertices[v3]->position - _vertices[v]->position).dot(_vertices[v2]->binormal)) > 0) {
                    // same side of the stroke
                    // now check for projective overlap: v3 and v4 onto v1 and v2
                    bool overlap_res = checkOverlap(v, v1, v2, v3, v4);
                    if (overlap_res) {
                        undecided_vertices.insert(v);
                        // std::cout << "triangle: " << t2[0] << " || " << t2[1] << " || " << t2[2] << std::endl;
                        // std::cout << "Vertex shared: " << v << "| v1: " << v1 << " | v2: " << v2 << " | v3: " << v3  << " | v4: " << v4 << std::endl;
                    }
                }

            }
        }
    }
    std::cout << "Vertex shared: overlap: " << undecided_vertices.size() << " out of total of " << _vertices.size() << " vertices." << std::endl;
    // std::cout << "Edge shared: " << undecided_edges.size() << " out of total of " << " edges." << std::endl;
    // combine all clusters and return it?
    for (auto edge : undecided_edges) {
        res.push_back(edgeToTriangles.at(edge));
    }
    for (auto vertex : undecided_vertices) {
        res.push_back(vertexToTriangles.at(vertex));
    }
    undecidedTriangles = res;


}


bool Mesh::checkOverlap(int v, int v1, int v2, int v3, int v4) {
    Vector3f v1v = _vertices[v]->position - _vertices[v1]->position;
    Vector3f v2v = _vertices[v]->position - _vertices[v2]->position;
    Vector3f normal = v1v.cross(v2v); // normal of the triangle plane
    // v3, v4 should be on the opposite side of v2 w.r.t. v1v
    Vector3f b_v1v = v1v.cross(normal); // binormal of v1v
    Vector3f b_v2v = v2v.cross(normal); // binormal of v2v
    // three cases, with v3 and v4 interchangeable
    Vector3f v1v3 = _vertices[v3]->position - _vertices[v1]->position;
    Vector3f v1v4 = _vertices[v4]->position - _vertices[v1]->position;
    Vector3f v1v2 = _vertices[v2]->position - _vertices[v1]->position;

    Vector3f v2v3 = _vertices[v3]->position - _vertices[v2]->position;
    Vector3f v2v4 = _vertices[v4]->position - _vertices[v2]->position;
    Vector3f v2v1 = _vertices[v1]->position - _vertices[v2]->position;

    int count_false = 0;
    // Case 1 -- if all four booleans are true
    // v3 same v2 w.r.t. v1v  --> v1v3 and v1v2 same direction --> v1v3.dot(b_v1v) same sign as v1v2.dot(b_v1v)
    bool v3_v2_v1v = v1v3.dot(b_v1v) * v1v2.dot(b_v1v) > 0; if (!v3_v2_v1v) count_false++;
    // v4 same v2 w.r.t. v1v
    bool v4_v2_v1v = v1v4.dot(b_v1v) * v1v2.dot(b_v1v) > 0; if (!v4_v2_v1v) count_false++;
    // v3 same v1 w.r.t.v2v
    bool v3_v1_v2v = v2v3.dot(b_v2v) * v2v1.dot(b_v2v) > 0; if (!v3_v1_v2v) count_false++;
    // v4 same v1 w.r.t.v2v
    bool v4_v1_v2v = v2v4.dot(b_v2v) * v2v1.dot(b_v2v) > 0; if (!v4_v1_v2v) count_false++;

    // case 1: true, true, true, true -- 0 false
    // case 2,3,4,5: only 1 false
    return count_false <= 1;
}




// --------------------- Get matches functions ------------------------
void Mesh::calculateTangentsAndBinormals(const vector<Vector3f> &vertices, const vector<Vector3f> &vertexNormals)
{
    // loop through all lines
    for (auto &line : _lines)
    {
        int n = line.size();
        // loop through all vertices on the line
        Vector3f first_tangent = (vertices[line[1]] - vertices[line[0]]).normalized(); // tangent of the first vertex is just the line segment direction
        _vertices[line[0]]->tangent = first_tangent;
        _vertices[line[0]]->binormal = (first_tangent.cross(vertexNormals[line[0]])).normalized();

        for (int i = 1; i < n - 1; i++)
        {
            Vector3f curA = vertices[line[i - 1]];
            Vector3f curB = vertices[line[i]];
            Vector3f curC = vertices[line[i + 1]];
            Vector3f AC = curC - curA;
            Vector3f B_normal = vertexNormals[line[i]];
            Vector3f AC_parallel = AC.dot(B_normal) / (B_normal.norm() * B_normal.norm()) * B_normal; // AC projected onto the direction of normal

            Vector3f cur_tangent = (AC - AC_parallel).normalized(); // tangent at B
            _vertices[line[i]]->tangent = cur_tangent;
            _vertices[line[i]]->binormal = (cur_tangent.cross(vertexNormals[line[i]])).normalized();
        }
        Vector3f last_tangent = (vertices[line[n - 1]] - vertices[line[n - 2]]).normalized(); // tangent of the last vertex is just the line segment direction
        _vertices[line[n - 1]]->tangent = last_tangent;
        _vertices[line[n - 1]]->binormal = (last_tangent.cross(vertexNormals[line[n - 1]])).normalized();
    }


}

float Mesh::vertexVertexScore(Vertex *P, Vertex *Q, bool leftside)
{
    Vector3f p = P->position;
    Vector3f q = Q->position;
    Vector3f tp = P->tangent;
    Vector3f tq = Q->tangent;
    Vector3f pb = (tp.cross(P->normal)).normalized();
    Vector3f qb = (tq.cross(Q->normal)).normalized();
    float da = (p - q).norm();
    float dt = 0.5 * (abs((p - q).dot(tp)) + abs((p - q).dot(tq)));
    float pStrokeWidth = P->strokeWidth;
    float qStrokeWidth = Q->strokeWidth;
    // This is specifically for a left match.
    Vector3f pc;
    if (leftside)
    {
        pc = p - pStrokeWidth * pb;
    }
    else
    {
        pc = p + pStrokeWidth * pb;
    }

    Vector3f ql = q - qStrokeWidth * qb;
    Vector3f qr = q + qStrokeWidth * qb;

    Vector3f qc;
    if ((ql - pc).norm() < (qr - pc).norm())
    {
        qc = ql;
    }
    else
    {
        qc = qr;
    }
    Vector3f mpqprime = 0.5 * (pc + qc);
    Vector3f mpq = 0.5 * (p + q);
    float dln = (mpq - mpqprime).norm();
    float sigma = 1.5f/2.f*(pStrokeWidth + qStrokeWidth);
    float finalscore = exp(-pow(da + dt + dln, 2) / (2.f * pow(sigma, 2.f)));
    return finalscore;
}

// Pi_1 mean P_(i+1)
float Mesh::persistenceScore(Vertex *Pi, Vertex *Qi, Vertex *Pi_1, Vertex *Qi_1)
{
    Vector3f pi_1 = Pi_1->position;
    Vector3f qi_1 = Qi_1->position;
    Vector3f pi = Pi->position;
    Vector3f qi = Qi->position;
    float pStrokeWidth = Pi->strokeWidth;
    float qStrokeWidth = Qi->strokeWidth;
    float dp = ((pi_1 - pi) - (qi_1 - qi)).norm() + ((pi_1 - qi) - (qi_1 - pi)).norm() + ((pi_1 - qi_1) - (pi - qi)).norm();

    float sigma = 1.5f/2.f*(pStrokeWidth + qStrokeWidth);
    float finalscore = exp(-pow(dp, 2) / (2.f * pow(sigma, 2.f)));
    return finalscore;
}

// Intermediate M from step i-1 to i
// pi_1 means p_(i-1)
float Mesh::computeM(int pi, int qi, int pi_1, int qi_1, bool leftSide)
{
    float vv = vertexVertexScore(_vertices[pi_1], _vertices[qi_1], leftSide);
    float pers = persistenceScore(_vertices[pi_1], _vertices[qi_1], _vertices[pi], _vertices[qi]);

    // naively punish inconsecutive qi and qi-1
    if (abs(qi - qi_1) > 1) return 1e-10*vv*pers;
    return vv * pers;

}

/**
 * @brief perform one viterbi on one stroke S to find the sequence of q's that maximizes the objective function of M_l
 * @param vector<Vertex*> S: stroke to be processed
 * @param vector<vector<Vertex*>> candidates: a vector of candidates vector<C(pi)> for each vertex pi in S
 * @return vector<Vertex*> Q: sequence of q's, each qi is the optimal match for each pi in S
 */
vector<int> Mesh::viterbi(vector<int> S, vector<vector<int>> candidates, bool leftSide)
{
    int k = 1;        // time index
    int K = S.size(); // number of steps = number of vertices in S
    int M = 0;        // max of number of candidates among all points pi
    for (int i = 0; i < candidates.size(); i++)
    {
        if (candidates[i].size() > M)
        {
            M = candidates[i].size();
        }
    }
    // if (M == 0) return {}; // if S does not have any potential matches on this side
    // construct a float[][] of size M*K
    float scores[M][K];
    // initialize the first column of dp to be all ones -- candidates for p0
    for (int i = 0; i < candidates[0].size(); i++)
    {
        scores[i][0] = 1.0;
    }
    // also need to store the current sequence of states at each q at time k -- an array of vectors, the array is of fixed size M
    vector<vector<int>> prev_sequences;
    vector<vector<int>> cur_sequences;
    // initialize the sequences to contain the starting points -- candidates for p0
    for (int i = 0; i < candidates[0].size(); i++)
    {
        prev_sequences.push_back({candidates[0][i]});
    }
    if (candidates[0].size() == 0)
    { // first node does not have candidate matches
        prev_sequences.push_back({-1});
    }
    // begin iterating through each step
    int prev_k = 0; // the previous step with valid states (candidates nonempty)
    while (k < K)
    {
        if (candidates[k].size() == 0)
        { // if the current node does not have candidate matches
            // add -1 to each prev_sequence to indicate that it does not have a matching vertex
            // then continue to next k without setting prev_k -- prev_k should be the last valid k
            if (prev_sequences.empty())
            {
                prev_sequences.push_back({-1});
            }
            else
            {
                for (auto &seq : prev_sequences)
                {
                    seq.push_back(-1);
                }
            }

            k++;
            continue;
        }
        cur_sequences = {};
        for (int cur = 0; cur < candidates[k].size(); cur++)
        {
            // for each current q, select the prev that maximizes M_score so far
            // for prev, its M_score is stored in scores[prev][k-1]
            float cur_max = -1e36;
            int max_prev = 0;
            for (int prev = 0; prev < candidates[prev_k].size(); prev++)
            {
                float stepM = computeM(S[k], candidates[k][cur], S[prev_k], candidates[prev_k][prev], leftSide);

                if (stepM * scores[prev][prev_k] > cur_max)
                {
                    cur_max = stepM * scores[prev][prev_k];
                    max_prev = prev;
                }
            }
            vector<int> newvec = prev_sequences[max_prev];
            newvec.push_back(candidates[k][cur]);
            cur_sequences.push_back(newvec); // extend from prev_seqence by appending cur to the mas prev sequence
            // use cur_max as the score for cur
            scores[cur][k] = cur_max;
        }
        // updaet prev_sequences to be cur_sequences: prev_sequences = cur_sequences
        prev_sequences = {};
        for (int i = 0; i < cur_sequences.size(); i++)
        {
            prev_sequences.push_back(cur_sequences[i]);
        }
        prev_k = k;
        k++;
    }
    // now select the q that has the maximum score and retrieve its sequence
    // all final scores are in scores[][K-1]
    int final_index = 0;
    float max_score = 0;
    for (int i = 0; i < prev_sequences.size(); i++)
    {
        if (scores[i][K - 1] > max_score)
        {
            max_score = scores[i][K - 1];
            final_index = i;
        }
    }
    return prev_sequences[final_index];
}

// populate the unordered_map<int, int> leftMatch and rightMatch
void Mesh::getMatches()
{
    // process each strip
    for (vector<int> S : _lines)
    {
        // convert the unordered_map<int, unordered_set<int>> leftRestrictedMatchingCandidates into two vector<vector<int>> candidates for S
        vector<vector<int>> left_candidates;
        vector<vector<int>> right_candidates;
        for (int point : S)
        {
            vector<int> point_left_candidates = {};
            if (leftRestrictedMatchingCandidates.contains(point))
            { // if it has some potential left matches
                point_left_candidates.insert(point_left_candidates.end(), leftRestrictedMatchingCandidates.at(point).begin(), leftRestrictedMatchingCandidates.at(point).end());
            }
            left_candidates.push_back(point_left_candidates);

            vector<int> point_right_candidates = {};
            if (rightRestrictedMatchingCandidates.contains(point))
            { // if it has some potential right matches
                point_right_candidates.insert(point_right_candidates.end(), rightRestrictedMatchingCandidates.at(point).begin(), rightRestrictedMatchingCandidates.at(point).end());
            }
            right_candidates.push_back(point_right_candidates);
        }
        // ========== check for previous matches and adjust candidate sets
        for (int i = 0; i < S.size(); i++) {
            int point = S[i];
            if (currentMatches.contains(point)) {
                for (int potential_match : currentMatches.at(point)) {
                    // see if it's on the left or right side of point using the binormal
                    Vector3f binormal = _vertices[point]->tangent.cross(_vertices[point]->normal);
                    bool hasRightBefore = false; bool hasLeftBefore = false;
                    // if it lines up with the binormal it's on right side, should go into right candidates
                    if ((_vertices[potential_match]->position - _vertices[point]->position).dot(binormal) > 0) { // right side
                        if (hasRightBefore) {
                            right_candidates[i].push_back(potential_match);
                        } else {
                            right_candidates[i] = {potential_match};
                            hasRightBefore = true;
                        }

                    }else {  // left side
                        if (hasLeftBefore) {
                            left_candidates[i].push_back(potential_match);
                        } else {
                            left_candidates[i] = {potential_match};
                            hasLeftBefore = true;
                        }
                    }
                }
            }
        }

        // starting finding matches for S
        vector<int> left_matches = viterbi(S, left_candidates, true);
        vector<int> right_matches = viterbi(S, right_candidates, false);

        // populate unordered_map<int, int> leftMatch and rightMatch
        for (int i = 0; i < S.size(); i++)
        {
            leftMatch.insert({S[i], left_matches[i]});
            rightMatch.insert({S[i], right_matches[i]});

            // ========= populate current matches
            if (currentMatches.contains(left_matches[i])) {
                currentMatches.at(left_matches[i]).push_back(S[i]);
            }else {
                currentMatches.insert({left_matches[i], {S[i]}});
            }
            if (currentMatches.contains(right_matches[i])) {
                currentMatches.at(right_matches[i]).push_back(S[i]);
            }else {
                currentMatches.insert({right_matches[i], {S[i]}});
            }
        }
    }
}

// -------- Get matching candidates functions -------------------------------------------------------------------------------

/**
 * @brief Mesh::splitStrokesIntoLeftRight
 * @param baseStrokeIndex
 * @return (LEFT strokes, RIGHT strokes)
 */
pair<vector<int>, vector<int>> Mesh::splitStrokesIntoLeftRight(int baseStrokeIndex)
{
    vector<int> leftStrokeIndices;
    vector<int> rightStrokeIndices;

    vector<int> baseStroke = this->_lines[baseStrokeIndex];
    int baseStrokeSize = baseStroke.size();
    int baseStrokeMidpoint = (int)((baseStrokeSize - (baseStrokeSize % 2)) / 2);
    Vertex *baseStrokeMidpointVertex = this->_vertices[baseStroke[baseStrokeMidpoint]];
    Vector3f baseBinormal = (baseStrokeMidpointVertex->tangent.cross(baseStrokeMidpointVertex->normal)).normalized();

    for (int i = 0; i < this->_lines.size(); i++)
    {
        if (i == baseStrokeIndex)
            continue;

        // get midpoint
        vector<int> currStroke = this->_lines[i];
        int currStrokeSize = currStroke.size();
        int currStrokeMidpoint = (int)((currStrokeSize - (currStrokeSize % 2)) / 2);
        int currStrokeMidpointInd = currStroke[currStrokeMidpoint];
        Vertex *currStrokeMidpointVertex = this->_vertices[currStrokeMidpointInd];
        Vector3f baseToCurrVec = (currStrokeMidpointVertex->position - baseStrokeMidpointVertex->position).normalized();

        float dotProduct = baseBinormal.dot(baseToCurrVec);
        if (dotProduct > 0)
        {
            rightStrokeIndices.push_back(i);
        }
        else
        {
            leftStrokeIndices.push_back(i);
        }
    }

    return make_pair(leftStrokeIndices, rightStrokeIndices);
}

bool areVerticesImmediateNeighbors(int pIndex, int qIndex, vector<int> stroke)
{
    for (int i = 0; i < stroke.size(); i++)
    {
        if (i == pIndex)
        {
            return qIndex == i - 1 || qIndex == i + 1;
        }
    }
    return false;
}

/**
 * @brief Mesh::intabs
 * @param x
 * @return the absolute value of x
 */
int intabs(int x){
    if(x<0){
        return -x;
    }
    return x;
}

/**
 * @brief Mesh::getnorm
 * @param p0
 * @param p1
 * @param p2
 * @return the normal of the plane formed by the three points
 */
Vector3f getnorm(Vector3f p0, Vector3f p1, Vector3f p2){
    Vector3f v01 = p1-p0;
    Vector3f v02 = p2-p0;
    Vector3f crossed = v01.cross(v02);
    return crossed/crossed.norm();
}


/**
 * @brief Mesh::triangulatePair
 * @param pi
 * @param qi
 * @param pn
 * @param qn
 * @return a list of triangles triangulating between the edges pi pn and qi qn
 */
std::vector<Vector3i> Mesh::triangulatePair(int pi,int qi,int pn, int qn){
    //So in practice, we want to push these new triangles back.
    //We need to be able to go through each pair of strokes, and iterate through all consecutive matches.

    std::vector<Vector3i> triangles = std::vector<Vector3i>();
    //qi and qi1 belong to the same stroke by default. Their difference should just be the number of points in between.
    //Check if qi and qn are the same
    if(qi == qn){
        triangles.push_back(Vector3i(pi,pn,qi));
    }
    else{
        //qi and qn are not the same
        int n_midpoints = intabs(qn-qi)-1;

        if(n_midpoints == 0){
            //qi and qn are consecutive

            //Get positions of all points
            Vector3f pip = _vertices[pi]->position;
            Vector3f qip = _vertices[qi]->position;
            Vector3f pnp = _vertices[pn]->position;
            Vector3f qnp = _vertices[qn]->position;

            //Try both triangulations. Choose the one with a larger dihedral angle.
            //Edit: Dihedral angle might not be great. Just do the one with the smaller diagonal difference.
            if((pip-qnp).norm()<(qip-pnp).norm()){
                triangles.push_back(Vector3i(pi,pn,qn));
                triangles.push_back(Vector3i(pi,qi,qn));
            }
            else{
                triangles.push_back(Vector3i(qi,qn,pn));
                triangles.push_back(Vector3i(qi,pi,pn));
            }
        }
        else{
            // //There are a nonzero number of midpoints
            // //For now, we choose a naive solution for triangulating. Splice along the approximate midpoint of the segment connecting qi to qn.
            int i_mid = static_cast<int>(floor(0.5*(static_cast<float>(qi)+static_cast<float>(qn))));

            int ibegin;
            int iend;
            if(qi < qn){
                ibegin = qi;
                iend = qn;
            }
            else{
                ibegin = qn;
                iend = qi;
            }

            // //Add above triangles
            for(int i = ibegin; i < i_mid; i++){
                triangles.push_back(Vector3i(pi,i+1,i));
            }
            //Add middle triangle
            //i_mid should be between qi and qn
            triangles.push_back(Vector3i(pi,pn,i_mid));
            //I HAVE A FEELING QI AND QN ARE NOT ON THE SAME STROKE

            // Add below triangles
            for(int i = i_mid; i < iend; i++){
                triangles.push_back(Vector3i(pn,i+1,i));
            }
        }
    }
    return triangles;
}


/**
 * @brief Mesh::doTwoVerticesMatch
 * @param p
 * @param q
 * @param leftside
 * @param isOnSameStroke - whether p & q are on the same stroke
 * @param strokeIndex - -1 if isOnSameStroke == false, stroke index if isOnSameStroke == true
 * @param closeToEnd - if either of these vertices are in the vicinity of stroke end-vertices
 * @return whether the vertices satisfy the condition
 */
bool Mesh::doTwoVerticesMatch(int pIndex, int qIndex, bool leftside, bool isOnSameStroke, int strokeIndex, bool closeToEnd)
{
    Vertex *p = this->_vertices[pIndex];
    Vertex *q = this->_vertices[qIndex];
    Vector3f pq = q->position - p->position;
//    Vector3f pBinormal = (p->tangent.cross(p->normal)).normalized();

    // condition 1
    float length = pq.norm();
    float pStrokeWidth = p->strokeWidth;
    float qStrokeWidth = q->strokeWidth;

    float sigma = 1.5f/2.f*(pStrokeWidth + qStrokeWidth);
    if (length > sigma)
    {
        return false;
    }

    // condition 2
    pq.normalize();
    float angle;
    if (leftside)
    {
        angle = acos(pq.dot(-p->binormal));
    }
    else
    {
        angle = acos(pq.dot(p->binormal));
    }
    if (angle > M_PI / 3.f)
    {
        return false;
    }

    // condition 3
    if (isOnSameStroke)
    {
        vector<int> stroke = this->_lines[strokeIndex];
        if (areVerticesImmediateNeighbors(pIndex, qIndex, stroke))
        {
            return false;
        }
    }

    // enforcing condition 2 for vertices in the vicinity of stroke end-vertices
    if (closeToEnd) {
        Vector3f qp = (q->position - p->position).normalized();
        float angle2; // angle between qp & q's binormal
        bool leftside2 = q->binormal.dot(qp) < 0; // whether p is on leftside of q
        if (leftside2) {
           angle2 = acos(qp.dot(-q->binormal));
        } else {
            angle2 = acos(qp.dot(q->binormal));
        }
        if (angle2 > M_PI / 3.f)
        {
            return false;
        }
    }

    return true;
}

// can optimize, by "caching" results
// return -1 if:
//  - matching frequency < 30%
//  - there is no pair of consecutive vertices on S that match a pair of consecutive vertices on T
int Mesh::calcNumberOfMatches(int baseStrokeIndex, int otherStrokeIndex, bool leftside)
{
    int baseCount = 0; // how many of base stroke's vertices got matched

    vector<int> baseStroke = this->_lines[baseStrokeIndex];
    vector<int> otherStroke = this->_lines[otherStrokeIndex];

    bool isConsecutivePair = false; // for 2nd returning -1 condition
    int prevBaseVertexIndex = -1; // index in stroke; for 2nd returning -1 condition

    for (int i = 0; i < baseStroke.size(); i++)
    {
        int baseVertex = baseStroke[i];
        bool isEverMatched = false;
        for (int j = 0; j < otherStroke.size(); j++)
        {
            // <notice, the 3 lines below are old comments>
            // TODO: IS THIS PART CORRECT?
            // !leftside is incorrect bc we don't know what side baseVertex is on w.r.t otherVertex
            // this is for p.9, section 5.2, restricted matching set subsection


            // TODO: FOR NOW, I SAY WITHIN THE LAST 5 VERTICES, BOTH V1 & V2 NEED TO HOLD CONDITION 2
            bool isEitherVertexCloseToEnd = (i > baseStroke.size() - this->settings->noOfNearEndVerticesToConsider || i < this->settings->noOfNearEndVerticesToConsider) || (j > otherStroke.size() - this->settings->noOfNearEndVerticesToConsider || j < this->settings->noOfNearEndVerticesToConsider);

            int otherVertex = otherStroke[j];
            bool baseOthermatched = this->doTwoVerticesMatch(baseVertex, otherVertex, leftside, false, -1, isEitherVertexCloseToEnd);

//            baseOthermatched = baseOthermatched && this->doTwoVerticesMatch(otherVertex, baseVertex, !leftside, false, -1);


            if (baseOthermatched)
            {
                isEverMatched = true;
                // this is not 2-way, because we will do the other way later when this func is called again
                if (!validVertexVertexMatch.contains(baseVertex))
                {
                    validVertexVertexMatch[baseVertex] = unordered_map<int, unordered_set<int>>();
                }
                if (!validVertexVertexMatch.at(baseVertex).contains(otherStrokeIndex))
                {
                    validVertexVertexMatch[baseVertex][otherStrokeIndex] = unordered_set<int>();
                }
                unordered_set<int> matches = validVertexVertexMatch.at(baseVertex).at(otherStrokeIndex);
                matches.insert(otherVertex);
                this->validVertexVertexMatch[baseVertex][otherStrokeIndex] = matches;

                // if we have not found any consecutive pair yet!
                // and baseVertex is not the first vertex on stroke
                // and prevVertex actually has any matches
                int prevBaseVertex = baseStroke[prevBaseVertexIndex];
                if (!isConsecutivePair && prevBaseVertexIndex != -1 && this->validVertexVertexMatch.contains(prevBaseVertex) && this->validVertexVertexMatch.at(prevBaseVertex).contains(otherStrokeIndex)) {
                    bool isBaseConsecutive = (i - prevBaseVertexIndex == 1);
                    unordered_set<int> prevBaseVertexMatches = this->validVertexVertexMatch.at(prevBaseVertex).at(otherStrokeIndex);
                    // this line below is saying:
                    // does the previous base vertex match with a consecutive neighbor of otherVertex?
                    bool isOtherConsecutive = prevBaseVertexMatches.contains(otherVertex - 1) || prevBaseVertexMatches.contains(otherVertex + 1);

                    isConsecutivePair = isBaseConsecutive && isOtherConsecutive;
                }
            }
        }
        if (isEverMatched)
        {
            baseCount++;
        }

        prevBaseVertexIndex = i;
    }

    float matchingFrequency = ((float)baseCount / (float)baseStroke.size()) > 0.3;

    baseCount = (matchingFrequency && isConsecutivePair) ? baseCount : -1;

    return baseCount;
}

void addedge(unordered_map<int, vector<std::pair<float,int>>>* adjacencymap, int i, int j, float v){
    if(adjacencymap->contains(i)){
        adjacencymap->at(i).push_back(std::make_pair(v,j));
    }
    else{
        vector<std::pair<float,int>> theset = vector<std::pair<float,int>>();
        theset.push_back(std::make_pair(v,j));
        adjacencymap->emplace(i , theset);
    }

    if(adjacencymap->contains(j)){
        adjacencymap->at(j).push_back(std::make_pair(v,i));
    }
    else{
        vector<std::pair<float,int>> theset = vector<std::pair<float,int>>();
        theset.push_back(std::make_pair(v,i));
        adjacencymap->emplace(j,theset);
    }
}

bool doTrianglesShareEdge(Vector3i t1, Vector3i t2){
    //What is the best way to check if they share an edge?
    int shared = 0;
    for(int i = 0; i < 3; i++){
        for(int j = 0; j < 3; j++){
            if(t1[i]==t2[j]){
                shared = shared + 1;
            }
        }
    }
    if(shared == 2){
        return true;
    }
    else if(shared >2){
        std::cout << "warning- two triangles share more than one edge in common" << std::endl;
        return true;
    }
    return false;
    //Now check all 9 edge pairs. Or uh. Find the vertex that corresponds to the other vertex. Maybe do that. Idk.
    //TODO: Implement this!
}

void Mesh::makeGraph(std::vector<Vector3i> trianglepatch){
//Chloe is making a map that points from triangle hashes to their incompatible neighbors
    unordered_map<int,unordered_set<int>> incompatibles;
    int numtriangles = trianglepatch.size();
    unordered_map<int, vector<std::pair<float,int>>> adjacencies;

    unordered_map<int,int> indicestohashes = unordered_map<int,int>();
    for(int i = 0; i < numtriangles; i++){
        //make a node for that triangle
        //Note: I don't think we even need a node structure.
        indicestohashes.emplace(i, tohash(trianglepatch[i],_vertices.size()));
    }
    //Also we need an output node. That will be below vvv
    indicestohashes.emplace(-1,-1);
    //Now look at...like.... ever single pair of triangles. F
    for(int i = 0; i < numtriangles; i++){
        for(int j = 0; i < numtriangles; j++){
            bool mutuallyincompatible = incompatibles.at(i).contains(j) || incompatibles.at(j).contains(i);
            //If they are mutually incompatible make an arc and give it a negative weight
            if(mutuallyincompatible){
                addedge(&adjacencies, i, j, -30.f);
            }
            else{
                bool commonedge = doTrianglesShareEdge(trianglepatch[i],trianglepatch[j]);
                //TODO: Compute if they share a common edge!
                //else: If they are not mutually incompatible but share a common edge give it a weight of 1
                if(commonedge){
                    addedge(&adjacencies, i, j, 1.f);
                }
            }
        }
    }
    //Make an edge between every triangle and the output node as well.
    for(int i = 0; i < numtriangles; i++){
        //Assign the weight based on the matching score
        float matchingweight = 0;
        addedge(&adjacencies, i, -1, matchingweight);
    }
}

// -------- PRIVATE ENDS -------------------------------------------------------------------------------
