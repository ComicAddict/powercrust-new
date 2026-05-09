/*
 * io_mesh.h -- multi-format mesh I/O for PowerCrust
 *
 * Input supported:
 *   .obj  ASCII Wavefront OBJ
 *   .ply  ASCII, binary little-endian, binary big-endian PLY
 *   .gii  GIFTI surface (ASCII, Base64Binary, GZipBase64Binary)
 *         GZip support requires zlib; compile with -DHAVE_ZLIB.
 *
 * Output supported (conversion from OFF):
 *   "obj"        ASCII Wavefront OBJ
 *   "ply_ascii"  ASCII PLY
 *   "ply_binary" Binary little-endian PLY
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/* Vertex positions read from a mesh file. */
typedef struct {
    double *x;
    double *y;
    double *z;
    int     n;   /* number of vertices */
} MeshVertices;

/*
 * read_mesh_vertices -- parse 3-D vertex coordinates from filename.
 * The file format is detected from the extension (.obj / .ply / .gii).
 * Returns the vertex count on success, 0 on failure.
 * The caller must free the result with free_mesh_vertices().
 */
int  read_mesh_vertices(const char *filename, MeshVertices *out);

/* Release memory allocated by read_mesh_vertices. */
void free_mesh_vertices(MeshVertices *mv);

/*
 * convert_off_file -- translate an OFF mesh to another format.
 *   format: "obj" | "ply_ascii" | "ply_binary"
 * Returns 1 on success, 0 on failure.
 */
int  convert_off_file(const char *off_file,
                      const char *out_file,
                      const char *format);

/*
 * convert_axis_off_file -- translate axis.off (mixed 2-vertex edges +
 * 3-vertex faces) to another format.
 *   PLY output: element edge (vertex1/vertex2) + element face (list).
 *   OBJ output: "l" line elements for edges, "f" for faces.
 *   format: "obj" | "ply_ascii" | "ply_binary"
 * Returns 1 on success, 0 on failure.
 */
int  convert_axis_off_file(const char *off_file,
                            const char *out_file,
                            const char *format);

#ifdef __cplusplus
}
#endif
