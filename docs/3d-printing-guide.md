# 3D プリンター製ロボット設計ガイド

草刈りロボットのフレーム・部品を 3D プリンターで製作するためのノウハウ集。

---

## 1. フィラメント選定

### 1.1 比較表

| 素材 | UV 耐性 | 耐熱温度 | 強度 | プリント難易度 | 屋外適性 | 価格 (1kg) |
|------|---------|---------|------|-------------|---------|-----------|
| **ASA** | ◎ 最良 | 105°C | ◎ | やや難 (反りやすい) | **最適** | ¥3,000〜4,000 |
| **PETG** | ○ 中程度 | 70〜80°C | ◎ | 易 | 良好 | ¥2,500〜3,500 |
| PLA | ✕ 劣化する | 55〜60°C | ○ | 最も易 | **不適** | ¥2,000〜3,000 |
| ABS | ○ 中程度 | 100°C | ◎ | 難 (反り・臭い) | 良好 | ¥2,500〜3,500 |
| TPU | ○ | 60〜80°C | △ (柔軟) | やや難 | ガスケット用 | ¥3,500〜5,000 |

### 1.2 推奨

- **フレーム・外装: ASA** — 直射日光に長期間さらされるパーツに最適。1 年間の屋外暴露テストで劣化なしの報告あり
  - 出典: [Wevolver: ASA vs PETG](https://www.wevolver.com), [3DTrcek: ASA vs PETG for Outdoor Use](https://3dtrcek.com)
- **内部構造・ブラケット: PETG** — プリントしやすく、強度も十分。UV に直接さらされない部品に
  - 出典: [i-maker.jp: PETGフィラメントの特性](https://i-maker.jp/blog/petg-filament-11214.html)
- **ガスケット・パッキン: TPU** — 柔軟性があり、防水シールに使える
  - 出典: [Reddit: Engineering Gaskets for 3D Prints](https://reddit.com/r/3Dprinting)
- **PLA は屋外ロボットに絶対に使わない** — 55°C で変形し始め、UV で脆化する

> **ASA のプリント注意点**: エンクロージャー (囲い) があるプリンターが望ましい。ベッド温度 95〜110°C、ノズル温度 240〜260°C。反り防止にブリムを使う。
> 出典: [Sovol3D: Choosing the Best Filament for Outdoor 3D Printing](https://sovol3d.com)

---

## 2. 強度を出すプリント設定

### 2.1 最も重要: 壁厚 (Wall) を増やす

**インフィルを上げるより壁の層数を増やす方が効果が大きい。**

| 設定項目 | 推奨値 | 根拠 |
|---------|--------|------|
| **外壁層数 (Wall Line Count)** | **4〜6 層** | インフィルより外壁が強度に支配的 |
| **壁厚** | **1.6〜2.4mm** (0.4mm ノズル × 4〜6) | 1.2mm 以上が目安 |
| インフィル密度 | **15〜20%** | 30% 以上は費用対効果が低い |
| インフィルパターン | **三角形 (Triangles)** or **ジャイロイド** | 等方性が高い |
| レイヤー高さ | **0.2mm** | 標準。防水を狙うなら 0.16mm |
| トップ/ボトム層数 | **5〜6 層** | 底面の強度確保 |

- 出典: [note: 3Dプリント部品の強度を出す設定](https://note.com/id_as_is/n/nf62f7b88081c)
- 出典: [Prusa Knowledge Base: インフィルパターン](https://help.prusa3d.com/ja/article/infill-patterns_177130)
- 出典: [RapidDirect: 3Dプリントパーツをより強くする方法](https://www.rapiddirect.com/ja/blog/how-to-make-3d-printed-parts-stronger/)

### 2.2 プリント方向

荷重がかかる方向と**レイヤーの積層方向**を考慮する。

```
✕ 悪い例: 引っ張り荷重が層間接着面に垂直
  ┌──────┐ ← 力
  │======│ ← レイヤー境界 (ここが剥がれる)
  │======│
  └──────┘

○ 良い例: 引っ張り荷重が層と平行
  ┌──────┐
  ║      ║ ← レイヤー境界
  ║      ║
  └──────┘
       ↑ 力
```

- **モーターマウント**: ネジ穴の軸がプリント方向と平行になるように配置
- **フレーム底板**: 水平にプリント (自然な向き)
- 出典: [QIDI Tech: 強力なパーツをプリントするためのヒント](https://jp.qidi3d.com/blogs/news/3d-printing-strong-parts)

### 2.3 設計のコツ

| コツ | 理由 | 出典 |
|------|------|------|
| 角を丸める (R2mm 以上) | 応力集中を避ける | [QIDI Tech](https://jp.qidi3d.com/blogs/news/3d-printing-strong-parts) |
| 高応力部はアルミ材と併用 | モーター軸受けなど | [G-Archiving: ロボットフレームの作り方](https://garchiving.com/robot-frame-design/) |
| フランジ (つば) をつける | 接合面積を増やして強度確保 | [RapidDirect](https://www.rapiddirect.com/ja/blog/how-to-make-3d-printed-parts-stronger/) |

---

## 3. ネジ穴: ヒートセットインサート

3D プリント部品にネジ止めする場合、**ヒートセットインサート (熱圧入ナット)** が最も信頼性が高い。

### 3.1 設計パラメータ

| 項目 | 推奨値 | 出典 |
|------|--------|------|
| 穴の深さ | インサート長 + 1〜2mm | [システムクリエイト](https://www.systemcreate-inc.co.jp/products/3d-printer/tips/insert-nut.html) |
| 穴の径 | インサート最小径よりわずかに大きく | [ボルテクノ](https://voltechno.com/blog/3dprinter-insert-nut/) |
| 周囲の壁厚 | 4〜6 層 (インフィルではなくソリッドで囲む) | [FacFox](https://facfox.com/docs/kb/how-to-design-and-install-heat-set-inserts-for-3d-printed-parts) |

### 3.2 埋め込み手順

```
1. はんだごての温度を「フィラメントのプリント温度 + 10〜20°C」に設定
   - PETG なら 250〜260°C、ASA なら 260〜270°C
2. 専用チップ (ヒートセットチップ) にインサートを載せる
3. 垂直にゆっくり圧入 (傾くと強度低下)
4. 90% 入ったらこてを抜き、平らな金属で押し切って面一に
```

- 出典: [DigiKey: 3Dプリントパーツへのインサート取り付け](https://www.digikey.jp/ja/articles/how-to-properly-install-heat-set-inserts-for-3d-printed-parts)
- 出典: [CNC Kitchen: Heat-set Inserts in 3D Printing](https://cnckitchen.com/blog/tipping-point-heat-set-inserts-in-3d-printing)
- 出典: [たねやつブログ: インサートナット埋め込みのコツ](https://taneyats.com/entry/3d-print-insert-nut-tips)

### 3.3 おすすめインサート

| サイズ | 用途 | Amazon で買える |
|--------|------|----------------|
| M3 × 5mm | LiDAR マウント、Pi ケース、小型ブラケット | ○ (50 個 ¥500〜800) |
| M4 × 6mm | モーターマウント、フレーム結合 | ○ |
| M5 × 8mm | 車軸、高荷重部 | ○ |

---

## 4. 防水設計

### 4.1 プリント設定で防水性を上げる

| 設定 | 値 | 理由 | 出典 |
|------|-----|------|------|
| 壁の層数 | 3 層以上 | 水の浸入経路を遮断 | [All3DP: How to 3D Print Waterproof Parts](https://all3dp.com) |
| レイヤー高さ | 0.16mm 以下 | 層間の隙間を最小化 | [All3DP](https://all3dp.com) |
| フロー率 | 102〜105% | 層間の微小V字隙間を埋める | [Sovol3D](https://sovol3d.com) |
| プリント温度 | +5〜10°C 高め | 層間接着を強化 | [Xometry](https://xometry.pro) |

### 4.2 ガスケット (パッキン)

| 方式 | 方法 | 適用 | 出典 |
|------|------|------|------|
| **TPU 直接プリント** | 蓋の合わせ面に TPU でガスケット溝をプリント | 簡易防水 | [Reddit r/3Dprinting](https://reddit.com/r/3Dprinting) |
| **シリコン型取り** | 3D プリントした型にシリコンを流し込み | 高品質シール | [Instructables](https://instructables.com) |
| **市販 O リング** | 溝を設計して市販品をはめる | 最も確実 | [3D Systems](https://3dsystems.com) |
| **シリコンシーラント** | 合わせ面にシリコンコーキングを塗布 | 最も手軽 | — |

### 4.3 後処理

| 方法 | 効果 | 出典 |
|------|------|------|
| エポキシ塗布 | 表面のレイヤー線を埋めて完全防水 | [JLC3DP](https://jlc3dp.com) |
| ウレタンスプレー | UV 保護 + 防水コーティング | [JLC3DP](https://jlc3dp.com) |
| アセトン蒸気 (ABS/ASA のみ) | 表面を溶かして平滑化 | [All3DP](https://all3dp.com) |

---

## 5. 振動対策

草刈り刃のモーター振動は 3D プリント部品の最大の敵。

| 対策 | 方法 | 出典 |
|------|------|------|
| **ゴムマウント** | モーターと筐体の間にゴムダンパーを挟む | [ReP_AL Maker Shop](https://repalmakershop.com) |
| **モーターサポート** | モーター端をブラケットで支え、たわみを防止 | [ReP_AL: Motor Supports Update (2024/07)](https://repalmakershop.com) |
| **重心を低く** | バッテリーを底面に配置し振動の影響を抑える | [The Mower Project](https://mowerproject.com) |
| **共振回避** | モーター回転数を段階的に上げず、共振点を跳び越す | [The Mower Project](https://mowerproject.com) |
| **刃のバランス** | ナイロンコードの長さを左右均等にする | [The Mower Project](https://mowerproject.com) |

---

## 6. 参考プロジェクトと STL データ

| プロジェクト | 内容 | STL 公開 | URL |
|-----------|------|---------|-----|
| **ReP_AL** | フル 3D プリント芝刈りロボット、Arduino ベース | ○ | [repalmakershop.com](https://repalmakershop.com) |
| **IndyMower #3** | 2024 年更新、BLDC モーター、モジュール設計 | ○ | [YouTube: Nikodem Bartnik](https://www.youtube.com/watch?v=SOfA9Yv_r_U) |
| **Thingiverse** | ロボット芝刈り機パーツ、トリマーヘッド多数 | ○ | [thingiverse.com/tag:robot_mower](https://www.thingiverse.com/tag:robot_mower) |
| **Printables** | ナイロンコードヘッド、モーターアダプター | ○ | [printables.com](https://www.printables.com/search/models?q=robot%20mower) |

---

## 7. grass-chopper 向け設計チェックリスト

### 部品一覧 (3D プリント)

| 部品 | 素材 | インフィル | 壁層数 | 注意点 |
|------|------|----------|--------|--------|
| 底板フレーム | ASA | 20% | 5 | 底面を上にしてプリント、ヒートセット M4 |
| 側板 × 4 | ASA | 15% | 4 | 合わせ面にガスケット溝 |
| モーターマウント × 2 | PETG | 30% | 6 | 高荷重、ゴムダンパー挟む |
| 刃モーターマウント | PETG | 40% | 6 | 振動対策必須、モーターサポート追加 |
| LiDAR マウント | PETG | 20% | 4 | ヒートセット M3、高さ調整可能に |
| Pi 5 ケース | ASA | 15% | 4 | 排熱穴 + 防水ルーバー |
| バッテリーホルダー | PETG | 20% | 4 | 重心を低く配置 |
| ナイロンコードヘッド | PETG | 50% | 6 | 高回転、バランス重要 |
| ガスケット | TPU | 100% | — | 柔軟素材、合わせ面に密着 |

### 合計フィラメント量の目安

| フィラメント | 必要量 | 費用 |
|------------|--------|------|
| ASA | 約 800g | ¥2,400〜3,200 |
| PETG | 約 500g | ¥1,250〜1,750 |
| TPU | 約 50g | ¥200〜300 |
| **合計** | **約 1.35kg** | **¥3,850〜5,250** |

板金加工 (¥30,000〜80,000) と比較して **1/6〜1/15 のコスト**。
