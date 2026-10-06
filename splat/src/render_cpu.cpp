#include "linesplat/render_cpu.hpp"

#include <algorithm>
#include <stdexcept>

namespace linesplat {

namespace {

template <typename T>
std::vector<GaussianGeomT<T>> scene_geometry(const GaussianScene& scene) {
  std::vector<GaussianGeomT<T>> g(static_cast<size_t>(scene.size()));
  for (int i = 0; i < scene.size(); ++i) g[size_t(i)] = gaussian_geometry<T>(scene, i);
  return g;
}

template <typename T>
std::vector<std::pair<LineSplatT<T>, int>> project_all(const std::vector<GaussianGeomT<T>>& geom,
                                                       const LineCameraT<T>& cam) {
  std::vector<std::pair<LineSplatT<T>, int>> hits;
  LineSplatT<T> s;
  for (size_t i = 0; i < geom.size(); ++i)
    if (project_to_line(cam, geom[i], &s)) hits.emplace_back(s, int(i));
  // Stable, so equal depths keep the Gaussian order, as the GPU's radix sort does.
  std::stable_sort(hits.begin(), hits.end(),
                   [](const std::pair<LineSplatT<T>, int>& a, const std::pair<LineSplatT<T>, int>& b) {
                     return a.first.depth < b.first.depth;
                   });
  return hits;
}

}  // namespace

template <typename T>
std::vector<std::pair<LineSplatT<T>, int>> project_scene(const GaussianScene& scene, const LineCameraT<T>& cam) {
  return project_all(scene_geometry<T>(scene), cam);
}

template <typename T>
LineImageT<T> render_lines_cpu(const GaussianScene& scene, const std::vector<LineCameraT<T>>& cams) {
  scene.validate();
  LineImageT<T> img;
  img.lines = int(cams.size());
  img.width = cams.empty() ? 0 : cams[0].width;
  img.channels = scene.num_features;
  for (const auto& c : cams)
    if (c.width != img.width) throw std::runtime_error("render_lines_cpu: all cameras need the same width");
  const int W = img.width, K = img.channels;
  img.values.assign(size_t(img.lines) * W * K, T(0));
  img.transmittance.assign(size_t(img.lines) * W, T(1));
  img.contributors.assign(size_t(img.lines) * W, 0);
  const std::vector<GaussianGeomT<T>> geom = scene_geometry<T>(scene);

#pragma omp parallel for schedule(dynamic, 4)
  for (int l = 0; l < img.lines; ++l) {
    const auto hits = project_all(geom, cams[size_t(l)]);
    // Bucket the hits by pixel (compressed rows), keeping depth order.
    std::vector<int> start(size_t(W) + 1, 0);
    for (const auto& h : hits)
      for (int p = h.first.p0; p < h.first.p1; ++p) ++start[size_t(p) + 1];
    for (int p = 0; p < W; ++p) start[size_t(p) + 1] += start[size_t(p)];
    std::vector<int> fill(start.begin(), start.end() - 1);
    std::vector<int> bucket(static_cast<size_t>(start[size_t(W)]));
    for (int h = 0; h < int(hits.size()); ++h)
      for (int p = hits[size_t(h)].first.p0; p < hits[size_t(h)].first.p1; ++p) bucket[size_t(fill[size_t(p)]++)] = h;

    std::vector<T> acc(static_cast<size_t>(K));
    for (int p = 0; p < W; ++p) {
      std::fill(acc.begin(), acc.end(), T(0));
      T trans = T(1);
      int used = 0;
      for (int b = start[size_t(p)]; b < start[size_t(p) + 1]; ++b) {
        const LineSplatT<T>& s = hits[size_t(bucket[size_t(b)])].first;
        const T a = splat_alpha_at(s.u, s.inv_var, s.alpha, p);
        if (a < T(kMinAlpha)) continue;
        const T next = trans * (T(1) - a);
        if (next < T(kMinTransmittance)) break;
        const float* f = &scene.features[size_t(hits[size_t(bucket[size_t(b)])].second) * K];
        const T w = a * trans;
        for (int c = 0; c < K; ++c) acc[size_t(c)] += T(f[c]) * w;
        trans = next;
        ++used;
      }
      const size_t px = size_t(l) * W + p;
      for (int c = 0; c < K; ++c) img.values[px * K + c] = acc[size_t(c)] + trans * T(scene.background[size_t(c)]);
      img.transmittance[px] = trans;
      img.contributors[px] = used;
    }
  }
  return img;
}

template <typename T>
LineImageT<T> features_to_bands(const GaussianScene& scene, const LineImageT<T>& in) {
  const int K = scene.num_features, B = scene.num_bands();
  if (in.channels != K) throw std::runtime_error("features_to_bands: channel count != num_features");
  LineImageT<T> out;
  out.lines = in.lines;
  out.width = in.width;
  out.channels = B;
  out.transmittance = in.transmittance;
  out.contributors = in.contributors;
  out.values.assign(size_t(in.lines) * in.width * B, T(0));
  const size_t px = size_t(in.lines) * in.width;
  for (size_t i = 0; i < px; ++i)
    for (int b = 0; b < B; ++b) {
      T s = T(0);
      for (int k = 0; k < K; ++k) s += T(scene.basis[size_t(b) * K + k]) * in.values[i * K + k];
      out.values[i * B + b] = s;
    }
  return out;
}

template LineImageT<float> render_lines_cpu(const GaussianScene&, const std::vector<LineCameraT<float>>&);
template LineImageT<double> render_lines_cpu(const GaussianScene&, const std::vector<LineCameraT<double>>&);
template LineImageT<float> features_to_bands(const GaussianScene&, const LineImageT<float>&);
template LineImageT<double> features_to_bands(const GaussianScene&, const LineImageT<double>&);
template std::vector<std::pair<LineSplatT<float>, int>> project_scene(const GaussianScene&, const LineCameraT<float>&);
template std::vector<std::pair<LineSplatT<double>, int>> project_scene(const GaussianScene&,
                                                                       const LineCameraT<double>&);

}  // namespace linesplat
