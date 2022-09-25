#ifndef CUBIC_SPLINE_PLANNER
#define CUBIC_SPLINE_PLANNER


#include <array>
#include <vector>

#define POINT_NUM_FOR_CUBLIC_SPLINE 100
/// xが戻らない場合の 3 次スプラインを計算する
/// CubeSplinePlan2D に呼ばれる
/// @param points <int,2>の array を要素に持つ点群vector
/// @return 点たちを3次スプライン補完した vector
std::vector<std::array<float, 2>> CublicSplinePlan(std::vector<std::array<int, 2>> const &points)
{
    std::vector<std::array<float, 2>> res;

    int n = points.size();
    std::vector<float> a(n), b(n - 1), c(n), d(n - 1), h(n - 1);

    // パラメータaiを求める
    for (int i = 0; i < n; i++)
    {
        a.at(i) = points.at(i).at(1);
    }

    // 各区間の変化量hiを求める
    for (int i = 0; i < (n - 1); i++)
    {
        h.at(i) = points.at(i + 1).at(0) - points.at(i).at(0);
    }

    // パラメータciを求める
    // まず行列式 Xc=Y の X
    std::vector<std::vector<float>> X(n, std::vector<float>(n, 0));
    for (int i = 1; i < n - 1; i++)
    { // 最初と最後の行は除く
        X.at(i).at(i - 1) = h.at(i - 1);
        X.at(i).at(i) = 2 * (h.at(i - 1) + h.at(i));
        X.at(i).at(i + 1) = h.at(i);
    }
    X.at(0).at(0) = X.at(n - 1).at(n - 1) = 1;

    // Y
    std::vector<float> Y(n, 0);
    for (int i = 1; i < n - 1; i++)
    {
        Y.at(i) = (a.at(i + 1) - a.at(i)) * 3 / h.at(i) - (a.at(i) - a.at(i - 1)) * 3 / h.at(i - 1);
    }

  
    // とりあえず Gauss-Seidel でとく
    // たぶんとける
    // 前進消去
    for (int k = 0; k < n - 1; k++)
    {
        float w = 1.0 / X.at(k).at(k);
        for (int i = k + 1; i < n; i++)
        {
            float M = X.at(i).at(k) * w;
            for (int j = k; j < n; j++)
            {
                X.at(i).at(j) -= M * X.at(k).at(j);
            }
            Y.at(i) -= M * Y.at(k);
        }
    }
    // 後退代入
    for (int i = n - 1; i >= 0; i--)
    {
        float tmp = 0;
        for (int j = i + 1; j < n; j++)
        {
            tmp += X.at(i).at(j) * c.at(j);
        }
        c.at(i) = (Y.at(i) - tmp) / X.at(i).at(i);
    }

    // パラメータdiを求める
    for (int i = 0; i < n - 1; i++)
    {
        d.at(i) = (c.at(i + 1) - c.at(i)) / (3 * h.at(i));
    }
    // パラメータbiを求める
    for (int i = 0; i < n - 1; i++)
    {
        b.at(i) = (a.at(i + 1) - a.at(i)) / h.at(i) - (h.at(i) * (c.at(i + 1) + 2 * c.at(i))) / 3;
    }
    // for (auto elem : b) {
    //     std::cout << elem << std::endl;
    // }

    // 補間
    float delta_x = float(points.at(n - 1).at(0) - points.at(0).at(0)) / POINT_NUM_FOR_CUBLIC_SPLINE;
    int ind = 0;
    for (float x = points.at(0).at(0); x < points.at(n - 1).at(0); x += delta_x)
    {
        if (x > points.at(ind + 1).at(0))
        {
            ind++;
            if (ind >= n)
                ind = n - 1;
        }
        float tmp = x - points.at(ind).at(0);
        res.push_back({{x, a.at(ind) + b.at(ind) * tmp + c.at(ind) * tmp * tmp + d.at(ind) * tmp * tmp * tmp}});
    }


    return res;
}

/// 媒介変数を用いて平面上の点たちを3次スプライン補間する。
/// 媒介変数の決定は工夫の余地があるがとりあえずリンク先のようにしてある。
/// http://www5d.biglobe.ne.jp/stssk/maze/spline.html
/// @param points <int,2>の array を要素に持つ点群vector
/// @return 点を3次スプライン補完した vector
std::vector<std::array<float, 2>> CublicSplinePlan2D(std::vector<std::array<int, 2>> const &points) {
    std::vector<std::array<float, 2>> res;
    int n = points.size();

    std::vector<std::array<int, 2>> x;
    std::vector<std::array<int, 2>> y;
    for (int i = 0; i < n; i++) {
        x.push_back({i, points.at(i).at(0)});
        y.push_back({i, points.at(i).at(1)});
    }
    std::vector<std::array<float, 2>> x_res = CublicSplinePlan(x);
    std::vector<std::array<float, 2>> y_res = CublicSplinePlan(y);
    for (int i = 0; i < POINT_NUM_FOR_CUBLIC_SPLINE; i++) {
        res.push_back({x_res.at(i).at(1), y_res.at(i).at(1)});
    }
    return res;
}
#endif