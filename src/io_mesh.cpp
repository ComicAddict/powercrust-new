/*
 * io_mesh.cpp -- multi-format mesh vertex reader / OFF converter
 *
 * Supported input formats
 *   OBJ  ASCII Wavefront OBJ  (.obj)
 *   PLY  ASCII, binary LE, binary BE  (.ply)
 *   GII  GIFTI surface: ASCII, Base64Binary, GZipBase64Binary  (.gii)
 *        GZip path requires zlib; compile with -DHAVE_ZLIB.
 *
 * Supported output (OFF converter)
 *   "obj"        ASCII Wavefront OBJ
 *   "ply_ascii"  ASCII PLY
 *   "ply_binary" Binary little-endian PLY
 */

#include "io_mesh.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#ifdef HAVE_ZLIB
#  include <zlib.h>
#endif

/* =========================================================
 * General utilities
 * ========================================================= */

static std::string str_trim(const std::string &s)
{
    size_t a = s.find_first_not_of(" \t\r\n");
    if (a == std::string::npos) return "";
    size_t b = s.find_last_not_of(" \t\r\n");
    return s.substr(a, b - a + 1);
}

static std::string str_lower(std::string s)
{
    std::transform(s.begin(), s.end(), s.begin(),
                   [](unsigned char c){ return (char)std::tolower(c); });
    return s;
}

static std::string file_ext(const char *filename)
{
    std::string fn(filename);
    size_t dot = fn.rfind('.');
    return (dot == std::string::npos) ? "" : str_lower(fn.substr(dot + 1));
}

static std::string read_whole_file(const char *filename)
{
    FILE *f = std::fopen(filename, "rb");
    if (!f) return "";
    std::fseek(f, 0, SEEK_END);
    long sz = std::ftell(f);
    std::fseek(f, 0, SEEK_SET);
    if (sz <= 0) { std::fclose(f); return ""; }
    std::string s(static_cast<size_t>(sz), '\0');
    std::fread(&s[0], 1, static_cast<size_t>(sz), f);
    std::fclose(f);
    return s;
}

static bool is_system_le()
{
    uint16_t v = 1;
    return *reinterpret_cast<uint8_t *>(&v) == 1;
}

/* =========================================================
 * OBJ reader (ASCII only — OBJ has no binary variant)
 * ========================================================= */

static int read_obj(const char *filename,
                    std::vector<double> &xs,
                    std::vector<double> &ys,
                    std::vector<double> &zs)
{
    FILE *f = std::fopen(filename, "r");
    if (!f) return 0;

    char line[1024];
    while (std::fgets(line, sizeof(line), f)) {
        if (line[0] == 'v' && line[1] == ' ') {
            double x, y, z;
            if (std::sscanf(line + 2, "%lf %lf %lf", &x, &y, &z) == 3) {
                xs.push_back(x);
                ys.push_back(y);
                zs.push_back(z);
            }
        }
    }
    std::fclose(f);
    return static_cast<int>(xs.size());
}

/* =========================================================
 * PLY reader (ASCII, binary LE, binary BE)
 * ========================================================= */

static int ply_type_bytes(const std::string &t)
{
    if (t=="char"||t=="int8"||t=="uchar"||t=="uint8")   return 1;
    if (t=="short"||t=="int16"||t=="ushort"||t=="uint16") return 2;
    if (t=="int"||t=="int32"||t=="uint"||t=="uint32"||
        t=="float"||t=="float32")                         return 4;
    if (t=="double"||t=="float64")                        return 8;
    return 0;
}

/* Read a typed binary value from buf, optionally byte-swapping. */
static double ply_read_val(const unsigned char *buf,
                            const std::string &type,
                            bool big_endian)
{
    auto bswap16 = [](uint16_t v) -> uint16_t {
        return (uint16_t)((v >> 8) | (v << 8));
    };
    auto bswap32 = [](uint32_t v) -> uint32_t {
        return ((v & 0xFF000000u) >> 24) | ((v & 0x00FF0000u) >> 8) |
               ((v & 0x0000FF00u) << 8)  | ((v & 0x000000FFu) << 24);
    };
    auto bswap64 = [](uint64_t v) -> uint64_t {
        v = ((v & 0xFF00FF00FF00FF00ULL) >> 8)  | ((v & 0x00FF00FF00FF00FFULL) << 8);
        v = ((v & 0xFFFF0000FFFF0000ULL) >> 16) | ((v & 0x0000FFFF0000FFFFULL) << 16);
        return (v >> 32) | (v << 32);
    };

    if (type=="float"||type=="float32") {
        uint32_t u; std::memcpy(&u, buf, 4);
        if (big_endian) u = bswap32(u);
        float fv; std::memcpy(&fv, &u, 4);
        return static_cast<double>(fv);
    }
    if (type=="double"||type=="float64") {
        uint64_t u; std::memcpy(&u, buf, 8);
        if (big_endian) u = bswap64(u);
        double dv; std::memcpy(&dv, &u, 8);
        return dv;
    }
    if (type=="int"||type=="int32") {
        uint32_t u; std::memcpy(&u, buf, 4);
        if (big_endian) u = bswap32(u);
        int32_t v; std::memcpy(&v, &u, 4);
        return static_cast<double>(v);
    }
    if (type=="uint"||type=="uint32") {
        uint32_t u; std::memcpy(&u, buf, 4);
        if (big_endian) u = bswap32(u);
        return static_cast<double>(u);
    }
    if (type=="short"||type=="int16") {
        uint16_t u; std::memcpy(&u, buf, 2);
        if (big_endian) u = bswap16(u);
        int16_t v; std::memcpy(&v, &u, 2);
        return static_cast<double>(v);
    }
    if (type=="ushort"||type=="uint16") {
        uint16_t u; std::memcpy(&u, buf, 2);
        if (big_endian) u = bswap16(u);
        return static_cast<double>(u);
    }
    if (type=="char"||type=="int8") {
        int8_t v; std::memcpy(&v, buf, 1);
        return static_cast<double>(v);
    }
    if (type=="uchar"||type=="uint8") {
        return static_cast<double>(buf[0]);
    }
    return 0.0;
}

struct PlyProp {
    std::string name;
    std::string type;        /* scalar type */
    int         bytes;       /* 0 if list */
    bool        is_list;
    std::string list_cnt_type;
    std::string list_val_type;
    int         list_cnt_bytes;
    int         list_val_bytes;
};

struct PlyElem {
    std::string          name;
    int                  count;
    std::vector<PlyProp> props;
};

static int read_ply(const char *filename,
                    std::vector<double> &xs,
                    std::vector<double> &ys,
                    std::vector<double> &zs)
{
    FILE *f = std::fopen(filename, "rb");
    if (!f) return 0;

    char line[512];

    /* Signature */
    if (!std::fgets(line, sizeof(line), f)) { std::fclose(f); return 0; }
    if (std::strncmp(line, "ply", 3) != 0)  { std::fclose(f); return 0; }

    /* Format */
    if (!std::fgets(line, sizeof(line), f)) { std::fclose(f); return 0; }
    std::string fmtline = str_lower(str_trim(std::string(line)));
    bool is_binary  = fmtline.find("binary")              != std::string::npos;
    bool big_endian = fmtline.find("binary_big_endian")   != std::string::npos;

    /* Header */
    std::vector<PlyElem> elems;
    PlyElem *cur = nullptr;

    while (std::fgets(line, sizeof(line), f)) {
        std::string l = str_trim(std::string(line));
        if (l == "end_header") break;

        if (l.substr(0,7) == "element") {
            elems.push_back(PlyElem());
            cur = &elems.back();
            char ename[128]; int ecount = 0;
            std::sscanf(l.c_str() + 8, "%127s %d", ename, &ecount);
            cur->name  = str_lower(ename);
            cur->count = ecount;

        } else if (l.substr(0,8) == "property" && cur) {
            PlyProp prop{};
            std::string rest = str_trim(l.substr(8));
            if (rest.substr(0,4) == "list") {
                prop.is_list = true;
                char cnt[32], val[32], nm[128];
                std::sscanf(rest.c_str() + 5, "%31s %31s %127s", cnt, val, nm);
                prop.list_cnt_type  = str_lower(cnt);
                prop.list_val_type  = str_lower(val);
                prop.list_cnt_bytes = ply_type_bytes(prop.list_cnt_type);
                prop.list_val_bytes = ply_type_bytes(prop.list_val_type);
                prop.name  = str_lower(nm);
                prop.bytes = 0;
            } else {
                char tp[32], nm[128];
                std::sscanf(rest.c_str(), "%31s %127s", tp, nm);
                prop.type  = str_lower(tp);
                prop.name  = str_lower(nm);
                prop.bytes = ply_type_bytes(prop.type);
            }
            cur->props.push_back(prop);
        }
    }

    /* Process vertex element */
    for (const auto &elem : elems) {
        if (elem.name != "vertex") continue;

        int xi = -1, yi = -1, zi = -1;
        for (int i = 0; i < (int)elem.props.size(); i++) {
            const auto &nm = elem.props[i].name;
            if (nm == "x") xi = i;
            else if (nm == "y") yi = i;
            else if (nm == "z") zi = i;
        }
        if (xi < 0 || yi < 0 || zi < 0) break;

        if (!is_binary) {
            /* ASCII PLY */
            for (int n = 0; n < elem.count; n++) {
                if (!std::fgets(line, sizeof(line), f)) break;
                std::vector<double> vals;
                char *p = line;
                while (*p) {
                    while (*p && std::isspace((unsigned char)*p)) ++p;
                    if (!*p) break;
                    double v; int nc;
                    if (std::sscanf(p, "%lf%n", &v, &nc) == 1) {
                        vals.push_back(v);
                        p += nc;
                    } else break;
                }
                if ((int)vals.size() > xi) xs.push_back(vals[xi]);
                if ((int)vals.size() > yi) ys.push_back(vals[yi]);
                if ((int)vals.size() > zi) zs.push_back(vals[zi]);
            }
        } else {
            /* Binary PLY */
            bool has_list = false;
            for (const auto &pr : elem.props)
                if (pr.is_list) { has_list = true; break; }

            if (!has_list) {
                /* Fixed-size records: compute byte offsets and row size */
                int row_bytes = 0;
                int x_off = 0, y_off = 0, z_off = 0;
                for (int pi = 0; pi < (int)elem.props.size(); pi++) {
                    if (pi == xi) x_off = row_bytes;
                    if (pi == yi) y_off = row_bytes;
                    if (pi == zi) z_off = row_bytes;
                    row_bytes += elem.props[pi].bytes;
                }
                std::vector<unsigned char> buf(static_cast<size_t>(row_bytes));
                for (int n = 0; n < elem.count; n++) {
                    if (std::fread(buf.data(), 1, row_bytes, f) !=
                            static_cast<size_t>(row_bytes)) break;
                    xs.push_back(ply_read_val(buf.data()+x_off,
                                              elem.props[xi].type, big_endian));
                    ys.push_back(ply_read_val(buf.data()+y_off,
                                              elem.props[yi].type, big_endian));
                    zs.push_back(ply_read_val(buf.data()+z_off,
                                              elem.props[zi].type, big_endian));
                }
            } else {
                /* Variable-length records (vertex elements rarely have lists,
                   but handle it anyway) */
                for (int n = 0; n < elem.count; n++) {
                    double xv = 0, yv = 0, zv = 0;
                    for (int pi = 0; pi < (int)elem.props.size(); pi++) {
                        const auto &pr = elem.props[pi];
                        if (!pr.is_list) {
                            unsigned char tmp[8] = {};
                            if (std::fread(tmp, 1, pr.bytes, f) !=
                                    static_cast<size_t>(pr.bytes)) goto done_ply;
                            double v = ply_read_val(tmp, pr.type, big_endian);
                            if (pi == xi) xv = v;
                            else if (pi == yi) yv = v;
                            else if (pi == zi) zv = v;
                        } else {
                            unsigned char cbuf[4] = {};
                            if (std::fread(cbuf, 1, pr.list_cnt_bytes, f) !=
                                    static_cast<size_t>(pr.list_cnt_bytes)) goto done_ply;
                            int cnt = (int)ply_read_val(cbuf,
                                                        pr.list_cnt_type, big_endian);
                            std::fseek(f, cnt * pr.list_val_bytes, SEEK_CUR);
                        }
                    }
                    xs.push_back(xv);
                    ys.push_back(yv);
                    zs.push_back(zv);
                }
                done_ply:;
            }
        }
        break; /* only need vertex element */
    }

    std::fclose(f);
    return static_cast<int>(xs.size());
}

/* =========================================================
 * Base-64 decoder
 * ========================================================= */

static const signed char B64[256] = {
    -1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1, /* 0-15  */
    -1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1, /* 16-31 */
    -1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,62,-1,-1,-1,63, /* 32-47 */
    52,53,54,55,56,57,58,59,60,61,-1,-1,-1,-1,-1,-1, /* 48-63 */
    -1, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9,10,11,12,13,14, /* 64-79 */
    15,16,17,18,19,20,21,22,23,24,25,-1,-1,-1,-1,-1, /* 80-95 */
    -1,26,27,28,29,30,31,32,33,34,35,36,37,38,39,40, /* 96-111*/
    41,42,43,44,45,46,47,48,49,50,51,-1,-1,-1,-1,-1  /* 112-127*/
    /* 128-255 all -1 (zero-initialised) */
};

static std::vector<unsigned char> base64_decode(const std::string &enc)
{
    std::vector<unsigned char> out;
    out.reserve(enc.size() * 3 / 4 + 4);
    int val = 0, bits = -8;
    for (unsigned char c : enc) {
        if (c == '\r' || c == '\n' || c == ' ' || c == '\t') continue;
        if (c == '=') break;
        signed char v = B64[c];
        if (v < 0) continue;
        val = (val << 6) | v;
        bits += 6;
        if (bits >= 0) {
            out.push_back((unsigned char)((val >> bits) & 0xFF));
            bits -= 8;
        }
    }
    return out;
}

/* =========================================================
 * zlib / gzip decompressor (used for GZipBase64 GIFTI)
 * ========================================================= */

static std::vector<unsigned char>
gzip_decompress(const std::vector<unsigned char> &compressed)
{
#ifdef HAVE_ZLIB
    z_stream strm{};
    /* windowBits = 15+32: auto-detect gzip/zlib header */
    if (inflateInit2(&strm, 15 + 32) != Z_OK) return {};

    strm.next_in  = const_cast<Bytef *>(compressed.data());
    strm.avail_in = static_cast<uInt>(compressed.size());

    std::vector<unsigned char> out;
    unsigned char chunk[65536];
    int ret;
    do {
        strm.next_out  = chunk;
        strm.avail_out = sizeof(chunk);
        ret = inflate(&strm, Z_NO_FLUSH);
        if (ret == Z_STREAM_ERROR || ret == Z_DATA_ERROR || ret == Z_MEM_ERROR)
            break;
        size_t have = sizeof(chunk) - strm.avail_out;
        out.insert(out.end(), chunk, chunk + have);
    } while (ret != Z_STREAM_END);

    inflateEnd(&strm);
    return out;
#else
    (void)compressed;
    std::fprintf(stderr,
        "io_mesh: GZipBase64 GIFTI requires zlib. "
        "Recompile with -DHAVE_ZLIB and link with -lz.\n");
    return {};
#endif
}

/* =========================================================
 * Minimal GIFTI XML helpers
 * ========================================================= */

/* Extract attribute value from an XML opening tag string. */
static std::string xml_attr(const std::string &tag, const std::string &attr)
{
    size_t pos = tag.find(attr + "=");
    if (pos == std::string::npos) return "";
    pos += attr.size() + 1;
    if (pos >= tag.size()) return "";
    char q = tag[pos];
    if (q != '"' && q != '\'') return "";
    ++pos;
    size_t end = tag.find(q, pos);
    if (end == std::string::npos) return "";
    return tag.substr(pos, end - pos);
}

static void bswap4(unsigned char *p)
{
    unsigned char t;
    t = p[0]; p[0] = p[3]; p[3] = t;
    t = p[1]; p[1] = p[2]; p[2] = t;
}

static void bswap8(unsigned char *p)
{
    for (int i = 0; i < 4; i++) {
        unsigned char t = p[i]; p[i] = p[7-i]; p[7-i] = t;
    }
}

/* =========================================================
 * GIFTI (.gii) reader
 *
 * The format is XML.  We search for the first DataArray element with
 * Intent containing "POINTSET", then decode its <Data> block
 * (ASCII / Base64Binary / GZipBase64Binary).
 *
 * Dimensionality and ArrayIndexingOrder determine how we unpack:
 *   RowMajorOrder    -> data layout: [x0,y0,z0, x1,y1,z1, ...]
 *                       Dim0 = num_vertices, Dim1 = 3
 *   ColumnMajorOrder -> data layout: [x0,x1,..., y0,y1,..., z0,z1,...]
 *                       Dim0 = 3, Dim1 = num_vertices
 * ========================================================= */

static int read_gii(const char *filename,
                    std::vector<double> &xs,
                    std::vector<double> &ys,
                    std::vector<double> &zs)
{
    std::string xml = read_whole_file(filename);
    if (xml.empty()) return 0;

    size_t pos = 0;
    while (true) {
        pos = xml.find("<DataArray", pos);
        if (pos == std::string::npos) break;

        /* Collect the full opening tag (may span multiple lines). */
        size_t tag_end = xml.find('>', pos);
        if (tag_end == std::string::npos) break;
        std::string tag = xml.substr(pos, tag_end - pos + 1);

        pos = tag_end + 1;

        if (xml_attr(tag, "Intent").find("POINTSET") == std::string::npos)
            continue;

        /* Parse relevant attributes */
        std::string encoding  = xml_attr(tag, "Encoding");
        std::string dtype     = xml_attr(tag, "DataType");
        std::string endian_s  = xml_attr(tag, "Endian");
        std::string order_s   = xml_attr(tag, "ArrayIndexingOrder");
        int dim0 = std::atoi(xml_attr(tag, "Dim0").c_str());
        int dim1 = std::atoi(xml_attr(tag, "Dim1").c_str());

        bool data_le   = (endian_s.find("Big")    == std::string::npos);
        bool row_major = (order_s.find("Column")  == std::string::npos);
        int  elem_sz   = 4; /* default: FLOAT32 */
        bool is_double = false;
        if (dtype.find("FLOAT64") != std::string::npos) {
            elem_sz = 8; is_double = true;
        } else if (dtype.find("INT16") != std::string::npos) {
            elem_sz = 2;
        }

        /* num_vertices and spatial dim from Dim0/Dim1 */
        int nv = row_major ? dim0 : dim1;
        if (nv <= 0) continue;

        /* Locate <Data> ... </Data> within this DataArray block. */
        size_t data_open  = xml.find("<Data>", tag_end);
        size_t data_close = xml.find("</Data>", tag_end);
        /* Verify they belong to this DataArray (before next DataArray). */
        size_t next_da = xml.find("<DataArray", pos);
        if (data_open  == std::string::npos) continue;
        if (data_close == std::string::npos) continue;
        if (next_da != std::string::npos && data_open > next_da) continue;

        std::string data_str = xml.substr(data_open + 6,
                                          data_close - (data_open + 6));
        data_str = str_trim(data_str);

        std::vector<double> coords;
        coords.reserve(static_cast<size_t>(nv) * 3);

        if (encoding == "ASCII") {
            const char *p = data_str.c_str();
            while (*p) {
                while (*p && std::isspace((unsigned char)*p)) ++p;
                if (!*p) break;
                double v; int n;
                if (std::sscanf(p, "%lf%n", &v, &n) == 1) {
                    coords.push_back(v);
                    p += n;
                } else break;
            }
        } else {
            /* Base64Binary or GZipBase64Binary */
            std::vector<unsigned char> raw = base64_decode(data_str);
            if (encoding == "GZipBase64Binary")
                raw = gzip_decompress(raw);

            bool need_swap = (data_le != is_system_le());
            size_t n_elems = raw.size() / static_cast<size_t>(elem_sz);
            coords.reserve(n_elems);

            for (size_t i = 0; i < n_elems; i++) {
                unsigned char buf[8];
                std::memcpy(buf,
                            raw.data() + i * static_cast<size_t>(elem_sz),
                            static_cast<size_t>(elem_sz));
                if (need_swap) {
                    if (elem_sz == 4) bswap4(buf);
                    else if (elem_sz == 8) bswap8(buf);
                }
                if (elem_sz == 4) {
                    float fv; std::memcpy(&fv, buf, 4);
                    coords.push_back(static_cast<double>(fv));
                } else if (elem_sz == 8 && is_double) {
                    double dv; std::memcpy(&dv, buf, 8);
                    coords.push_back(dv);
                } else {
                    uint16_t u16; std::memcpy(&u16, buf, 2);
                    coords.push_back(static_cast<double>((int16_t)u16));
                }
            }
        }

        /* Unpack into xs / ys / zs */
        if (row_major) {
            /* [x0,y0,z0, x1,y1,z1, ...] */
            for (int i = 0; i < nv; i++) {
                size_t base = static_cast<size_t>(i) * 3;
                if (base + 2 >= coords.size()) break;
                xs.push_back(coords[base]);
                ys.push_back(coords[base + 1]);
                zs.push_back(coords[base + 2]);
            }
        } else {
            /* [x0,x1,...,xN, y0,y1,...,yN, z0,z1,...,zN] */
            for (int i = 0; i < nv; i++) {
                size_t n = static_cast<size_t>(nv);
                if (static_cast<size_t>(i) + 2*n >= coords.size()) break;
                xs.push_back(coords[static_cast<size_t>(i)]);
                ys.push_back(coords[static_cast<size_t>(i) + n]);
                zs.push_back(coords[static_cast<size_t>(i) + 2*n]);
            }
        }
        break; /* processed the first POINTSET array */
    }

    return static_cast<int>(xs.size());
}

/* =========================================================
 * Public C API — read_mesh_vertices / free_mesh_vertices
 * ========================================================= */

extern "C" {

int read_mesh_vertices(const char *filename, MeshVertices *out)
{
    if (!filename || !out) return 0;

    std::vector<double> xs, ys, zs;
    std::string ext = file_ext(filename);

    int n = 0;
    if      (ext == "obj") n = read_obj(filename, xs, ys, zs);
    else if (ext == "ply") n = read_ply(filename, xs, ys, zs);
    else if (ext == "gii") n = read_gii(filename, xs, ys, zs);
    else {
        std::fprintf(stderr,
            "io_mesh: unrecognised extension '%s' in '%s'. "
            "Expected .obj / .ply / .gii\n", ext.c_str(), filename);
        return 0;
    }

    if (n <= 0) return 0;

    out->n = n;
    out->x = static_cast<double *>(std::malloc(n * sizeof(double)));
    out->y = static_cast<double *>(std::malloc(n * sizeof(double)));
    out->z = static_cast<double *>(std::malloc(n * sizeof(double)));
    if (!out->x || !out->y || !out->z) {
        std::free(out->x); std::free(out->y); std::free(out->z);
        out->x = out->y = out->z = nullptr;
        out->n = 0;
        return 0;
    }

    std::memcpy(out->x, xs.data(), n * sizeof(double));
    std::memcpy(out->y, ys.data(), n * sizeof(double));
    std::memcpy(out->z, zs.data(), n * sizeof(double));
    return n;
}

void free_mesh_vertices(MeshVertices *mv)
{
    if (!mv) return;
    std::free(mv->x); std::free(mv->y); std::free(mv->z);
    mv->x = mv->y = mv->z = nullptr;
    mv->n = 0;
}

/* =========================================================
 * OFF reader (internal helper for convert_off_file)
 * ========================================================= */

struct OffMesh {
    std::vector<double>             verts; /* packed xyz */
    std::vector<std::vector<int>>   faces;
};

static bool read_off(const char *filename, OffMesh &mesh)
{
    FILE *f = std::fopen(filename, "r");
    if (!f) return false;

    char line[512];
    /* Skip "OFF" header line */
    while (std::fgets(line, sizeof(line), f)) {
        std::string l = str_trim(std::string(line));
        if (l.empty() || l[0] == '#') continue;
        if (l.substr(0,3) == "OFF") break;
    }

    /* Read counts */
    int nv = 0, nf = 0, ne = 0;
    while (std::fgets(line, sizeof(line), f)) {
        std::string l = str_trim(std::string(line));
        if (l.empty() || l[0] == '#') continue;
        std::sscanf(l.c_str(), "%d %d %d", &nv, &nf, &ne);
        break;
    }

    mesh.verts.resize(static_cast<size_t>(nv) * 3);
    for (int i = 0; i < nv; i++) {
        while (std::fgets(line, sizeof(line), f)) {
            std::string l = str_trim(std::string(line));
            if (l.empty() || l[0] == '#') continue;
            std::sscanf(l.c_str(), "%lf %lf %lf",
                        &mesh.verts[i*3], &mesh.verts[i*3+1], &mesh.verts[i*3+2]);
            break;
        }
    }

    for (int i = 0; i < nf; i++) {
        while (std::fgets(line, sizeof(line), f)) {
            std::string l = str_trim(std::string(line));
            if (l.empty() || l[0] == '#') continue;
            int k = 0;
            std::sscanf(l.c_str(), "%d", &k);
            std::vector<int> face(static_cast<size_t>(k));
            const char *p = l.c_str();
            /* skip k */
            while (*p && !std::isspace((unsigned char)*p)) ++p;
            for (int j = 0; j < k; j++) {
                while (*p && std::isspace((unsigned char)*p)) ++p;
                int idx = 0; int nc = 0;
                std::sscanf(p, "%d%n", &idx, &nc);
                face[j] = idx;
                p += nc;
            }
            mesh.faces.push_back(face);
            break;
        }
    }

    std::fclose(f);
    return true;
}

/* =========================================================
 * Public C API — convert_off_file
 * ========================================================= */

int convert_off_file(const char *off_file,
                     const char *out_file,
                     const char *format)
{
    if (!off_file || !out_file || !format) return 0;

    OffMesh mesh;
    if (!read_off(off_file, mesh)) {
        std::fprintf(stderr, "io_mesh: cannot read OFF file '%s'\n", off_file);
        return 0;
    }

    int nv = (int)(mesh.verts.size() / 3);
    int nf = (int)mesh.faces.size();
    std::string fmt(format);

    if (fmt == "obj") {
        FILE *f = std::fopen(out_file, "w");
        if (!f) return 0;
        std::fprintf(f, "# Converted from %s by PowerCrust\n", off_file);
        for (int i = 0; i < nv; i++)
            std::fprintf(f, "v %.12g %.12g %.12g\n",
                         mesh.verts[i*3], mesh.verts[i*3+1], mesh.verts[i*3+2]);
        for (const auto &face : mesh.faces) {
            std::fprintf(f, "f");
            for (int idx : face) std::fprintf(f, " %d", idx + 1); /* OBJ is 1-based */
            std::fprintf(f, "\n");
        }
        std::fclose(f);
        return 1;

    } else if (fmt == "ply_ascii") {
        FILE *f = std::fopen(out_file, "w");
        if (!f) return 0;
        std::fprintf(f, "ply\nformat ascii 1.0\n");
        std::fprintf(f, "element vertex %d\n", nv);
        std::fprintf(f, "property double x\nproperty double y\nproperty double z\n");
        std::fprintf(f, "element face %d\n", nf);
        std::fprintf(f, "property list uchar int vertex_indices\n");
        std::fprintf(f, "end_header\n");
        for (int i = 0; i < nv; i++)
            std::fprintf(f, "%.12g %.12g %.12g\n",
                         mesh.verts[i*3], mesh.verts[i*3+1], mesh.verts[i*3+2]);
        for (const auto &face : mesh.faces) {
            std::fprintf(f, "%d", (int)face.size());
            for (int idx : face) std::fprintf(f, " %d", idx);
            std::fprintf(f, "\n");
        }
        std::fclose(f);
        return 1;

    } else if (fmt == "ply_binary") {
        FILE *f = std::fopen(out_file, "wb");
        if (!f) return 0;
        /* Write ASCII header */
        std::fprintf(f, "ply\nformat binary_little_endian 1.0\n");
        std::fprintf(f, "element vertex %d\n", nv);
        std::fprintf(f, "property float x\nproperty float y\nproperty float z\n");
        std::fprintf(f, "element face %d\n", nf);
        std::fprintf(f, "property list uchar int vertex_indices\n");
        std::fprintf(f, "end_header\n");
        /* Write binary data (LE float32 vertices, uchar+int32 faces) */
        for (int i = 0; i < nv; i++) {
            for (int k = 0; k < 3; k++) {
                float fv = static_cast<float>(mesh.verts[i*3+k]);
                std::fwrite(&fv, sizeof(float), 1, f);
            }
        }
        for (const auto &face : mesh.faces) {
            uint8_t cnt = static_cast<uint8_t>(face.size());
            std::fwrite(&cnt, 1, 1, f);
            for (int idx : face) {
                int32_t v = static_cast<int32_t>(idx);
                std::fwrite(&v, sizeof(int32_t), 1, f);
            }
        }
        std::fclose(f);
        return 1;
    }

    std::fprintf(stderr,
        "io_mesh: unknown output format '%s'. "
        "Use 'obj', 'ply_ascii', or 'ply_binary'.\n", format);
    return 0;
}

/* =========================================================
 * convert_axis_off_file -- convert axis.off (mixed 2/3-vertex OFF) to PLY/OBJ.
 *
 * axis.off has:
 *   - standard OFF header + vertex block
 *   - primitive block mixing "2 a b" edge lines and "3 a b c" face lines
 *
 * PLY output uses element edge (vertex1/vertex2) + element face (list).
 * OBJ  output uses "l a b" line elements + "f a b c" face elements.
 * ========================================================= */

int convert_axis_off_file(const char *off_file,
                           const char *out_file,
                           const char *format)
{
    if (!off_file || !out_file || !format) return 0;

    FILE *f = std::fopen(off_file, "r");
    if (!f) {
        std::fprintf(stderr, "io_mesh: cannot read axis OFF file '%s'\n", off_file);
        return 0;
    }

    char line[512];

    /* Skip to OFF header line */
    while (std::fgets(line, sizeof(line), f)) {
        std::string l = str_trim(std::string(line));
        if (l.empty() || l[0] == '#') continue;
        if (l.substr(0, 3) == "OFF") break;
    }

    /* Read counts: nv = vertices, np = total primitives (edges + faces) */
    int nv = 0, np = 0, dummy = 0;
    while (std::fgets(line, sizeof(line), f)) {
        std::string l = str_trim(std::string(line));
        if (l.empty() || l[0] == '#') continue;
        std::sscanf(l.c_str(), "%d %d %d", &nv, &np, &dummy);
        break;
    }

    /* Read vertices */
    std::vector<double> verts(static_cast<size_t>(nv) * 3);
    for (int i = 0; i < nv; i++) {
        while (std::fgets(line, sizeof(line), f)) {
            std::string l = str_trim(std::string(line));
            if (l.empty() || l[0] == '#') continue;
            std::sscanf(l.c_str(), "%lf %lf %lf",
                        &verts[i*3], &verts[i*3+1], &verts[i*3+2]);
            break;
        }
    }

    /* Read primitives, split into edges and triangles */
    std::vector<std::pair<int,int>>  edges;
    std::vector<std::vector<int>>    faces;

    for (int i = 0; i < np; i++) {
        while (std::fgets(line, sizeof(line), f)) {
            std::string l = str_trim(std::string(line));
            if (l.empty() || l[0] == '#') continue;
            int k = 0;
            std::sscanf(l.c_str(), "%d", &k);
            if (k == 2) {
                int a = 0, b = 0;
                std::sscanf(l.c_str(), "%*d %d %d", &a, &b);
                edges.push_back({a, b});
            } else if (k >= 3) {
                std::vector<int> face(static_cast<size_t>(k));
                const char *p = l.c_str();
                while (*p && !std::isspace((unsigned char)*p)) ++p;
                for (int j = 0; j < k; j++) {
                    while (*p && std::isspace((unsigned char)*p)) ++p;
                    int idx = 0, nc = 0;
                    std::sscanf(p, "%d%n", &idx, &nc);
                    face[j] = idx;
                    p += nc;
                }
                faces.push_back(face);
            }
            break;
        }
    }
    std::fclose(f);

    std::string fmt(format);

    if (fmt == "ply_ascii") {
        FILE *out = std::fopen(out_file, "w");
        if (!out) return 0;
        std::fprintf(out, "ply\nformat ascii 1.0\n");
        std::fprintf(out, "element vertex %d\n", nv);
        std::fprintf(out, "property double x\nproperty double y\nproperty double z\n");
        if (!edges.empty()) {
            std::fprintf(out, "element edge %d\n", (int)edges.size());
            std::fprintf(out, "property int vertex1\nproperty int vertex2\n");
        }
        if (!faces.empty()) {
            std::fprintf(out, "element face %d\n", (int)faces.size());
            std::fprintf(out, "property list uchar int vertex_indices\n");
        }
        std::fprintf(out, "end_header\n");
        for (int i = 0; i < nv; i++)
            std::fprintf(out, "%.12g %.12g %.12g\n",
                         verts[i*3], verts[i*3+1], verts[i*3+2]);
        for (const auto &e : edges)
            std::fprintf(out, "%d %d\n", e.first, e.second);
        for (const auto &face : faces) {
            std::fprintf(out, "%d", (int)face.size());
            for (int idx : face) std::fprintf(out, " %d", idx);
            std::fprintf(out, "\n");
        }
        std::fclose(out);
        return 1;

    } else if (fmt == "ply_binary") {
        FILE *out = std::fopen(out_file, "wb");
        if (!out) return 0;
        std::fprintf(out, "ply\nformat binary_little_endian 1.0\n");
        std::fprintf(out, "element vertex %d\n", nv);
        std::fprintf(out, "property float x\nproperty float y\nproperty float z\n");
        if (!edges.empty()) {
            std::fprintf(out, "element edge %d\n", (int)edges.size());
            std::fprintf(out, "property int vertex1\nproperty int vertex2\n");
        }
        if (!faces.empty()) {
            std::fprintf(out, "element face %d\n", (int)faces.size());
            std::fprintf(out, "property list uchar int vertex_indices\n");
        }
        std::fprintf(out, "end_header\n");
        for (int i = 0; i < nv; i++) {
            for (int k = 0; k < 3; k++) {
                float fv = static_cast<float>(verts[i*3+k]);
                std::fwrite(&fv, sizeof(float), 1, out);
            }
        }
        for (const auto &e : edges) {
            int32_t a = static_cast<int32_t>(e.first);
            int32_t b = static_cast<int32_t>(e.second);
            std::fwrite(&a, sizeof(int32_t), 1, out);
            std::fwrite(&b, sizeof(int32_t), 1, out);
        }
        for (const auto &face : faces) {
            uint8_t cnt = static_cast<uint8_t>(face.size());
            std::fwrite(&cnt, 1, 1, out);
            for (int idx : face) {
                int32_t v = static_cast<int32_t>(idx);
                std::fwrite(&v, sizeof(int32_t), 1, out);
            }
        }
        std::fclose(out);
        return 1;

    } else if (fmt == "obj") {
        FILE *out = std::fopen(out_file, "w");
        if (!out) return 0;
        std::fprintf(out, "# Converted from %s by PowerCrust\n", off_file);
        for (int i = 0; i < nv; i++)
            std::fprintf(out, "v %.12g %.12g %.12g\n",
                         verts[i*3], verts[i*3+1], verts[i*3+2]);
        for (const auto &e : edges)
            std::fprintf(out, "l %d %d\n", e.first + 1, e.second + 1);
        for (const auto &face : faces) {
            std::fprintf(out, "f");
            for (int idx : face) std::fprintf(out, " %d", idx + 1);
            std::fprintf(out, "\n");
        }
        std::fclose(out);
        return 1;
    }

    std::fprintf(stderr,
        "io_mesh: unknown axis output format '%s'. "
        "Use 'obj', 'ply_ascii', or 'ply_binary'.\n", format);
    return 0;
}

} /* extern "C" */
