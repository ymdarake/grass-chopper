# 関連プロジェクト・事例調査レポート

## 1. 主要な類似プロジェクト

### 1.1 OpenMower / OpenMowerNext

**最も成熟した OSS 草刈りロボットプロジェクト**

| 項目 | 内容 |
|------|------|
| URL | github.com/ClemensElflein/OpenMower |
| ROS 2 版 | github.com/jkaflik/OpenMowerNext (Jazzy 移植) |
| ハードウェア | YardForce Classic 500 を改造、Pi 4/5 + Pico、xESC BLDC ドライバ |
| 測位 | RTK-GPS (u-blox F9P)、ペリメーターワイヤー不要 |
| コミュニティ | Discord で活発、Version 2 ハードウェア発表済み |

**grass-chopper に活用できるアイデア:**

- **xESC ドライバ**: OpenMower の xESC mini / xESC 2040 は BLDC モーターの FOC 制御に特化。草刈り刃モーターに最適。自前で ESC を設計する必要がなくなる
- **Pi 5 + Pico アーキテクチャ**: grass-chopper の Phase 5 設計案と完全に一致。OpenMower のカスタム基板 (メインボード PCB) の回路図が公開されており、参考にできる
- **OpenMowerNext の ROS 2 Jazzy 移植**: 我々と同じ ROS 2 Jazzy + Nav2 構成。彼らのコードからハードウェアインターフェースの実装パターンを参考にできる

---

### 1.2 SmartMower

**2025年発のROS 2 Jazzyネイティブ草刈りプロジェクト**

| 項目 | 内容 |
|------|------|
| URL | discourse.ros.org/t/smartmower-a-ros-2-jazzy-mower-project |
| ハードウェア | Pi 5 + Pico、超音波センサー + カメラ |
| 特徴 | Nav2 統合、障害物検知を重視 |

**grass-chopper に活用できるアイデア:**

- **超音波センサー (JSN-SR04T) の併用**: LiDAR が苦手なガラスや黒い物体を超音波で補完する発想。屋外では防水型の JSN-SR04T を追加することで検知漏れを軽減
- **Web UI でのミッション管理**: SmartMower はブラウザから走行領域を指定・監視できる Web UI を計画。grass-chopper の mission_tree_node に Web フロントエンドを追加する拡張として面白い

---

### 1.3 Ardumower / ROSMower

**Arduino ベースの老舗草刈りロボット + ROS 移植**

| 項目 | 内容 |
|------|------|
| URL | ardumower.de |
| ハードウェア | Arduino Mega、BLDC モーター、36V Li-ion |
| ROS 版 | コミュニティが「ROSMower」として Pi + ROS に移植中 |
| コミュニティ | フォーラムが充実、ハードウェア BOM が詳細 |

**grass-chopper に活用できるアイデア:**

- **スイングブレード安全機構**: Ardumower は石に当たると刃が跳ね上がる「スイング式」ブレードを採用。モーター保護 + 安全性向上。固定刃より実用的
- **36V Li-ion バッテリーシステム**: 3S LiPo (11.1V) より高電圧の 36V システム。モーターの効率が向上し、配線の電流が下がる。ただし BMS が複雑になる
- **電流センサー (Mauch PL-200-HV)**: 高精度な電力監視。`battery_sim_node` の実機版で参考になる

---

### 1.4 Roktrack2 (国内プロジェクト)

**ソーラー駆動・AI搭載の自作草刈り機**

| 項目 | 内容 |
|------|------|
| URL | protopedia.net/prototype/5339 |
| ハードウェア | Pi 4 / Pi 3A+、ソーラーパネル |
| 特徴 | 人検知・害虫検知・植生監視、ソーラー駆動 |

**grass-chopper に活用できるアイデア:**

- **ソーラーパネルによる充電**: ドッキングステーションにソーラーパネルを設置すれば、電源のない屋外でも自律充電が可能。`mission_tree_node` の CHARGING 状態をそのまま活用できる
- **AI 物体検出 (人・動物)**: カメラ画像で人や動物を検出し、刃を緊急停止。`incline_monitor` と同じ twist_mux 最優先パターンで統合可能

---

### 1.5 Netosa (国内ブログ)

**ROS 2 ベースの草刈りロボット開発シリーズ**

| 項目 | 内容 |
|------|------|
| URL | netosa.com |
| ハードウェア | カスタム Turtlebot3 ベース |
| 特徴 | 屋外ナビゲーション・マッピングの技術検討記事群 |

**grass-chopper に活用できるアイデア:**

- **Turtlebot3 ベースでのプロトタイピング**: 実機の前に市販の Turtlebot3 で Nav2 + SLAM の屋外テストを行う方法。ハードウェア自作のリスクを減らせる
- **屋外 SLAM の失敗パターン共有**: 草原での SLAM 破綻、GPS 信号のマルチパスなど、実体験ベースの知見

---

## 2. 安全設計で取り入れるべきアイデア

各プロジェクトの安全設計から、grass-chopper に追加すべき機能:

### 2.1 ハードウェア安全層 (ソフトウェアに頼らない)

| 機能 | 実装方法 | 参考プロジェクト |
|------|---------|----------------|
| **物理 E-Stop ボタン** | ラッチ式ボタンでモーター電源を直接遮断。ソフトウェア不要 | OpenMower, Ardumower |
| **リフトセンサー** | マイクロスイッチで持ち上げを検知 → 刃モーター即停止 | OpenMower |
| **バンパーセンサー** | 機械式バンパー → GPIO 割り込みで cmd_vel = 0 | SmartMower |

> **重要**: ソフトウェア E-Stop (ROS ノード) は**ハードウェア E-Stop の補完であり、代替ではない**。ROS が死んでもモーターが止まる設計が必須。

### 2.2 環境センサー

| 機能 | センサー | ROS 統合 | 参考プロジェクト |
|------|---------|---------|----------------|
| **雨検知** | 静電容量式雨センサー (¥500) | `rain_detected` トピック → mission_tree で RETURNING に遷移 | OpenMower, Growbotics |
| **超音波距離** | JSN-SR04T (防水) × 2〜4 | collision_monitor の追加データソース | SmartMower |
| **温度監視** | DS18B20 (モーター温度) | 過熱で刃モーター停止 | Ardumower |

### 2.3 ソフトウェア安全層 (grass-chopper で追加可能)

| 機能 | 実装案 | 工数 |
|------|--------|------|
| **雨検知 → 自動帰還** | `mission_behaviors.py` に `should_return_rain()` 追加、mission_tree で状態遷移 | 小 |
| **リフト検知 → 刃停止** | `safety_monitor_node` 新規作成、twist_mux priority: 40 | 小 |
| **通信タイムアウト → 全停止** | Pico との UART が N 秒途絶えたら cmd_vel = 0 | 小 |
| **Watchdog タイマー** | Pico 側で ROS からの指令が途絶えたらモーター停止 | 小 |

---

## 3. カバレッジ計画の改善候補

| ライブラリ | 特徴 | grass-chopper との関係 |
|-----------|------|---------------------|
| **Fields2Cover** | 学術的カバレッジ計画ライブラリ。Boustrophedon, Spiral, Back-and-Forth 対応 | `coverage_planner.py` の `Polygon → list[Waypoint]` インターフェースを維持したまま移行可能 |
| **ipa_coverage_planning** | ROS パッケージ。OccupancyGrid ベースの分割カバレッジ | `map_region_detector.py` の出力をそのまま入力にできる |

> **現状判断**: grass-chopper の Boustrophedon 実装は 27 テストで検証済み。Fields2Cover への移行は「あれば良い」レベルで、実機で動いてからでも遅くない。

---

## 4. grass-chopper の差別化ポイント

他プロジェクトと比較して grass-chopper が既に持っている強み:

| 観点 | OpenMower | SmartMower | grass-chopper |
|------|-----------|------------|--------------|
| ROS 2 Jazzy ネイティブ | 移植中 | 開発中 | **完了** |
| テスト | 少 | 少 | **212 件** |
| Humble Object パターン | なし | なし | **8 モジュール分離** |
| ミッション管理 | シンプル | 計画段階 | **フルステートマシン** |
| ドッキング | ドックに戻る | 未実装 | **AprilTag + opennav_docking** |
| 傾斜検知 | なし | なし | **IMU + フィルタ + ヒステリシス** |
| CI/CD | なし | なし | **GitHub Actions** |
| シミュレーション環境 | 限定的 | なし | **7 ワールド** |

**grass-chopper はシミュレーション完成度とテスト網羅性で他プロジェクトを大きく上回っている。** 実機移行時には OpenMower のハードウェア設計を参考にしつつ、ソフトウェア品質の優位性を維持するのが最善。

---

## 5. 実機移行時に取り入れるべきアイデア (優先順)

| 優先度 | アイデア | 参考元 | 追加コスト |
|--------|---------|--------|-----------|
| **必須** | 物理 E-Stop ボタン (ラッチ式) | OpenMower, Ardumower | ¥500 |
| **必須** | リフトセンサー (マイクロスイッチ) | OpenMower | ¥300 |
| **必須** | Watchdog (Pico 側タイムアウト停止) | Ardumower | ¥0 (ファームウェア) |
| **推奨** | 雨センサー + 自動帰還ロジック | OpenMower, Growbotics | ¥500 |
| **推奨** | スイングブレード (石衝突保護) | Ardumower | ¥2,000 |
| **推奨** | 超音波センサー × 2 (LiDAR 補完) | SmartMower | ¥1,500 |
| **検討** | ソーラー充電ステーション | Roktrack2 | ¥10,000〜 |
| **検討** | Web UI (ミッション管理画面) | SmartMower | ¥0 (開発コスト) |
| **将来** | カメラ物体検出 (人・動物) | Roktrack2 | Jetson 必要 |
| **将来** | Fields2Cover 移行 | OpenMowerNext | ¥0 (開発コスト) |
