/* attrib.h — EAGL "AttribSys" reader for NFSU2 GLOBAL data.
 *
 * GlobalB.lzc is JDLZ-compressed; GLOBALB.BUN is its already-decompressed form
 * (parse the .BUN directly — no JDLZ codec needed). GLOBALB.BUN is a flat
 * sequence of 0x00135200 attribute records, each following the standard
 * (magic u32 LE, size u32 LE) chunk header. Layout of a record payload,
 * reverse-engineered from the "ALUMINUM" block:
 *
 *   +0x00 u32   flags/type
 *   +0x04 u32   count
 *   +0x08 u32   count
 *   +0x0C u32   CLASS HASH   (e.g. 0x2E65E067)
 *   +0x10 u32   count
 *   +0x14 char  instance NAME, NUL-padded ("ALUMINUM", "FALKEN TIRES", ...)
 *   ....        float[] value array (multipliers/params: grip, slip, offsets)
 *
 * First cut: enumerate records, pull the name, class hash, and the trailing
 * float array (heuristic: finite floats after the name field). Enough to dump
 * every tire/wheel record and diff Stock vs Pro to locate grip/slip columns.
 * Precise per-field hashes come next once we know which columns matter. */
#ifndef N2_ATTRIB_H
#define N2_ATTRIB_H
#include <stdint.h>
#include <string.h>
#include <math.h>

#define N2_ATTR_MAGIC 0x00135200u

typedef struct {
    long      off;         /* file offset of the record's payload */
    uint32_t  size;        /* payload size */
    uint32_t  classhash;   /* 32-bit class hash (u32 at name-0x08) */
    char      name[48];    /* instance name string */
    const float *vals;     /* trailing float array (points into the file buffer) */
    int       nvals;       /* how many trailing floats look valid */
} N2Attrib;

static uint32_t n2a_u32(const unsigned char *p){
    return p[0] | p[1]<<8 | p[2]<<16 | (uint32_t)p[3]<<24;
}
static int n2a_is_finite_val(float v){
    return v==v && v>-1e18f && v<1e18f;   /* reject NaN/inf/garbage */
}

/* Decode one 0x00135200 record at payload [off,off+size). Returns 1 on success. */
static int n2_attrib_decode(const unsigned char *d, long off, uint32_t size, N2Attrib *a){
    memset(a, 0, sizeof *a);
    a->off = off; a->size = size;
    /* first printable ASCII run >= 4 chars is the instance name */
    long ni = -1, nlen = 0;
    for (long i = 0; i + 4 <= (long)size; i++){
        int ok = 1;
        for (int k = 0; k < 4; k++){ int c = d[off+i+k]; if (c < 0x20 || c > 0x7e){ ok = 0; break; } }
        if (ok){ long j = i; while (j < (long)size){ int c = d[off+j]; if (c < 0x20 || c > 0x7e) break; j++; }
            ni = i; nlen = j - i; break; }
    }
    if (ni < 0) return 0;
    int L = nlen < 47 ? (int)nlen : 47;
    memcpy(a->name, d+off+ni, L); a->name[L] = 0;
    if (ni >= 8) a->classhash = n2a_u32(d+off+ni-8);   /* class hash sits before the name */
    /* float array: after the name, aligned up to 4, collect finite floats to EOF */
    long fp = (off + ni + nlen + 3) & ~3L;
    a->vals = (const float *)(d + fp);
    int n = 0;
    for (long p = fp; p + 4 <= off + size; p += 4){
        float v; memcpy(&v, d+p, 4);
        if (!n2a_is_finite_val(v)) break;   /* stop at first non-float (padding/hash) */
        n++;
    }
    a->nvals = n;
    return 1;
}

/* Walk every 0x00135200 record. Calls cb(a, user) for each. Returns record count. */
static int n2_attrib_walk(const unsigned char *d, long len,
                          void (*cb)(const N2Attrib *, void *), void *user){
    long o = 0; int count = 0;
    while (o + 8 <= len){
        uint32_t id = n2a_u32(d+o), size = n2a_u32(d+o+4);
        long payload = o + 8;
        if (id == N2_ATTR_MAGIC && payload + size <= len){
            N2Attrib a;
            if (n2_attrib_decode(d, payload, size, &a)){ if (cb) cb(&a, user); count++; }
        } else if (id == 0 || payload + size > len) {
            break;   /* not a well-formed record stream past here */
        }
        o = payload + size;
    }
    return count;
}

#endif /* N2_ATTRIB_H */
