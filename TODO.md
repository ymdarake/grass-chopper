# TODO

## 優先度 高 (Phase 5 ソフトウェア先行実装)

- [x] CI/CD (GitHub Actions) — `make test-pure` + `make check-syntax` を PR で自動実行
- [x] 傾斜フィルタリング (ローパスフィルタ + ヒステリシス) — 純粋関数で TDD
- [x] IMU キャリブレーション関数 (取り付けオフセット補正) — 純粋関数で TDD
- [x] Pico 通信プロトコル + PID 制御 — 純粋ロジック TDD (33テスト)
- [ ] Pico ファームウェア (MicroPython) — 上記ロジックを実機に移植
- [ ] ros2_control hardware_interface — Pico との UART 通信スケルトン (モック付き TDD)
- [ ] robot_launch.py — 実機用 launch ファイル (sim_launch.py から Gazebo/Bridge 除去)
- [ ] シャーシの 3D モデル — OpenSCAD でパラメトリック設計、STL 出力

## 優先度 中

- [ ] Nav2 プランナー比較検証 (DWB, MPPI)
- [ ] 螺旋カバレッジパターン
- [ ] Fields2Cover 移行検討
- [ ] RViz2 可視化整備

## メモ: 畝 (うね) 対策

畑で利用する場合、畝 (幅 60cm 程度、傾斜あり) は LiDAR では検知しにくい (水平スキャンが上を通過)。

**対策:**
1. **事前に走行禁止領域を設定** — `coverage_params.yaml` に畝の位置を障害物ポリゴンとして登録。`coverage_planner.py` が畝の間だけを走行する経路を自動生成 (既存機能で対応可能)
2. **IMU 傾斜検知のセーフティネット** — 畝に乗り上げた場合 `incline_monitor` が検知して停止 (実装済み)
3. **(将来) RTK-GPS + 高精度地図** — cm 精度で畝の位置を避けられる
4. **(将来) 深度カメラ (RealSense 等)** — 地面の凹凸を 3D で検知 (高価)
