// SPDX-License-Identifier: MIT
#include "gicp15d/point_cloud_io.h"

#include <cstdio>
#include <cstring>
#include <fstream>
#include <limits>
#include <map>
#include <stdexcept>
#include <unordered_map>

namespace gicp15d {
namespace {

// Minimal, dependency-free sha256 (FIPS 180-4). 64 hex chars.
struct Sha256
{
  std::uint32_t h[8] = {0x6a09e667u, 0xbb67ae85u, 0x3c6ef372u, 0xa54ff53au,
    0x510e527fu, 0x9b05688cu, 0x1f83d9abu, 0x5be0cd19u};
  std::uint64_t total = 0;
  unsigned char buf[64];
  std::size_t buf_len = 0;

  static std::uint32_t rotr(std::uint32_t x, int n) {return (x >> n) | (x << (32 - n));}

  void block(const unsigned char * p)
  {
    static const std::uint32_t k[64] = {
      0x428a2f98u, 0x71374491u, 0xb5c0fbcfu, 0xe9b5dba5u, 0x3956c25bu, 0x59f111f1u,
      0x923f82a4u, 0xab1c5ed5u, 0xd807aa98u, 0x12835b01u, 0x243185beu, 0x550c7dc3u,
      0x72be5d74u, 0x80deb1feu, 0x9bdc06a7u, 0xc19bf174u, 0xe49b69c1u, 0xefbe4786u,
      0x0fc19dc6u, 0x240ca1ccu, 0x2de92c6fu, 0x4a7484aau, 0x5cb0a9dcu, 0x76f988dau,
      0x983e5152u, 0xa831c66du, 0xb00327c8u, 0xbf597fc7u, 0xc6e00bf3u, 0xd5a79147u,
      0x06ca6351u, 0x14292967u, 0x27b70a85u, 0x2e1b2138u, 0x4d2c6dfcu, 0x53380d13u,
      0x650a7354u, 0x766a0abbu, 0x81c2c92eu, 0x92722c85u, 0xa2bfe8a1u, 0xa81a664bu,
      0xc24b8b70u, 0xc76c51a3u, 0xd192e819u, 0xd6990624u, 0xf40e3585u, 0x106aa070u,
      0x19a4c116u, 0x1e376c08u, 0x2748774cu, 0x34b0bcb5u, 0x391c0cb3u, 0x4ed8aa4au,
      0x5b9cca4fu, 0x682e6ff3u, 0x748f82eeu, 0x78a5636fu, 0x84c87814u, 0x8cc70208u,
      0x90befffau, 0xa4506cebu, 0xbef9a3f7u, 0xc67178f2u};
    std::uint32_t w[64];
    for (int i = 0; i < 16; i++) {
      w[i] = (std::uint32_t(p[4 * i]) << 24) | (std::uint32_t(p[4 * i + 1]) << 16) |
        (std::uint32_t(p[4 * i + 2]) << 8) | std::uint32_t(p[4 * i + 3]);
    }
    for (int i = 16; i < 64; i++) {
      const std::uint32_t s0 = rotr(w[i - 15], 7) ^ rotr(w[i - 15], 18) ^ (w[i - 15] >> 3);
      const std::uint32_t s1 = rotr(w[i - 2], 17) ^ rotr(w[i - 2], 19) ^ (w[i - 2] >> 10);
      w[i] = w[i - 16] + s0 + w[i - 7] + s1;
    }
    std::uint32_t a = h[0], b = h[1], c = h[2], d = h[3], e = h[4], f = h[5], g = h[6], hh = h[7];
    for (int i = 0; i < 64; i++) {
      const std::uint32_t s1 = rotr(e, 6) ^ rotr(e, 11) ^ rotr(e, 25);
      const std::uint32_t ch = (e & f) ^ ((~e) & g);
      const std::uint32_t t1 = hh + s1 + ch + k[i] + w[i];
      const std::uint32_t s0 = rotr(a, 2) ^ rotr(a, 13) ^ rotr(a, 22);
      const std::uint32_t maj = (a & b) ^ (a & c) ^ (b & c);
      const std::uint32_t t2 = s0 + maj;
      hh = g; g = f; f = e; e = d + t1; d = c; c = b; b = a; a = t1 + t2;
    }
    h[0] += a; h[1] += b; h[2] += c; h[3] += d; h[4] += e; h[5] += f; h[6] += g; h[7] += hh;
  }

  void update(const unsigned char * p, std::size_t n)
  {
    total += n;
    while (n > 0) {
      const std::size_t take = std::min(n, std::size_t(64) - buf_len);
      std::memcpy(buf + buf_len, p, take);
      buf_len += take;
      p += take;
      n -= take;
      if (buf_len == 64) {block(buf); buf_len = 0;}
    }
  }

  std::string finish()
  {
    const std::uint64_t bits = total * 8;
    const unsigned char pad = 0x80;
    update(&pad, 1);
    const unsigned char zero = 0x00;
    while (buf_len != 56) {update(&zero, 1);}
    unsigned char len[8];
    for (int i = 0; i < 8; i++) {len[7 - i] = static_cast<unsigned char>((bits >> (8 * i)) & 0xffu);}
    total -= 8;  // length bytes are not part of the message
    update(len, 8);
    char out[65];
    for (int i = 0; i < 8; i++) {std::snprintf(out + 8 * i, 9, "%08x", h[i]);}
    return std::string(out, 64);
  }
};
}  // namespace

CloudPtr loadXyzBin(const std::string & path, std::string * error)
{
  auto fail = [&](const std::string & what) -> CloudPtr {
      if (error) {*error = what;}
      return nullptr;
    };
  std::ifstream f(path, std::ios::binary);
  if (!f) {return fail("OPEN_FAILED");}
  std::uint64_t n = 0;
  f.read(reinterpret_cast<char *>(&n), sizeof(n));
  if (!f || n == 0) {return fail("EMPTY_OR_BAD_HEADER");}
  // 20M point hard ceiling: protects against a corrupt header allocating 240 GB.
  if (n > 20000000ull) {return fail("POINT_COUNT_ABOVE_SAFETY_LIMIT");}
  std::vector<float> raw(static_cast<std::size_t>(n) * 3);
  f.read(reinterpret_cast<char *>(raw.data()), static_cast<std::streamsize>(raw.size() * sizeof(float)));
  if (!f) {return fail("TRUNCATED_POINT_DATA");}
  auto cloud = std::make_shared<Cloud>();
  cloud->resize(static_cast<std::size_t>(n));
  for (std::size_t i = 0; i < n; i++) {
    const float x = raw[3 * i + 0], y = raw[3 * i + 1], z = raw[3 * i + 2];
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
      return fail("NONFINITE_POINT");
    }
    cloud->point(i) << double(x), double(y), double(z), 1.0;
  }
  return cloud;
}

CloudPtr loadXyzBinChecked(
  const std::string & path, std::size_t min_points, std::size_t max_points,
  std::string * error)
{
  auto cloud = loadXyzBin(path, error);
  if (!cloud) {return nullptr;}
  if (cloud->size() < min_points) {
    if (error) {*error = "TOO_FEW_POINTS";}
    return nullptr;
  }
  if (cloud->size() > max_points) {
    if (error) {*error = "TOO_MANY_POINTS";}
    return nullptr;
  }
  return cloud;
}

std::string fileSha256(const std::string & path)
{
  std::ifstream f(path, std::ios::binary);
  if (!f) {return std::string();}
  Sha256 sha;
  std::vector<char> buf(1 << 20);
  while (f) {
    f.read(buf.data(), static_cast<std::streamsize>(buf.size()));
    const std::streamsize got = f.gcount();
    if (got > 0) {
      sha.update(reinterpret_cast<const unsigned char *>(buf.data()), static_cast<std::size_t>(got));
    }
  }
  return sha.finish();
}

CloudPtr voxelDownsample(const Cloud & cloud, double leaf)
{
  auto out = std::make_shared<Cloud>();
  if (leaf <= 0) {
    out->resize(cloud.size());
    for (std::size_t i = 0; i < cloud.size(); i++) {out->point(i) = cloud.point(i);}
    return out;
  }
  struct Acc {Eigen::Vector3d sum = Eigen::Vector3d::Zero(); std::size_t n = 0;};
  std::unordered_map<std::int64_t, Acc> voxels;
  voxels.reserve(cloud.size() / 2 + 16);
  const double inv = 1.0 / leaf;
  for (std::size_t i = 0; i < cloud.size(); i++) {
    const Eigen::Vector3d p = cloud.point(i).head<3>();
    const std::int64_t ix = static_cast<std::int64_t>(std::floor(p.x() * inv));
    const std::int64_t iy = static_cast<std::int64_t>(std::floor(p.y() * inv));
    const std::int64_t iz = static_cast<std::int64_t>(std::floor(p.z() * inv));
    const std::int64_t key = (ix * 73856093LL) ^ (iy * 19349669LL) ^ (iz * 83492791LL);
    auto & a = voxels[key];
    a.sum += p;
    a.n += 1;
  }
  out->resize(voxels.size());
  std::size_t k = 0;
  for (const auto & kv : voxels) {
    const Eigen::Vector3d c = kv.second.sum / double(kv.second.n);
    out->point(k) << c.x(), c.y(), c.z(), 1.0;
    k++;
  }
  return out;
}

CloudStats cloudStats(const Cloud & cloud)
{
  CloudStats s;
  s.n = cloud.size();
  if (cloud.empty()) {return s;}
  s.min = Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity());
  s.max = Eigen::Vector3d::Constant(-std::numeric_limits<double>::infinity());
  for (std::size_t i = 0; i < cloud.size(); i++) {
    const Eigen::Vector3d p = cloud.point(i).head<3>();
    if (!p.allFinite()) {s.nonfinite++; continue;}
    s.min = s.min.cwiseMin(p);
    s.max = s.max.cwiseMax(p);
    s.mean += p;
  }
  const double denom = double(std::max<std::size_t>(1, cloud.size() - s.nonfinite));
  s.mean /= denom;
  return s;
}

}  // namespace gicp15d
