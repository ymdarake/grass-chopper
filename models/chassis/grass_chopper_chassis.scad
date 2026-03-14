/**
 * ============================================================================
 * grass-chopper シャーシ (OpenSCAD パラメトリックモデル)
 * ============================================================================
 * URDF (robot_description.urdf.xacro) の寸法と一致させた
 * 3D プリント用シャーシモデル。
 *
 * URDF との対応:
 *   - chassis: 300×200×100mm (URDF: 0.3×0.2×0.1m)
 *   - 駆動輪: X=-75mm, Y=±120mm (シャーシ外側)
 *   - キャスター: X=+100mm (前方)
 *   - wheel_separation: 240mm
 *
 * 使い方:
 *   1. OpenSCAD でこのファイルを開く
 *   2. パラメータを調整
 *   3. F6 (レンダリング) → File → Export as STL
 *
 * プリント設定 (推奨):
 *   - 素材: ASA (屋外) or PETG (屋内テスト)
 *   - 壁厚: 4〜6 層 (0.4mm ノズルなら 1.6〜2.4mm)
 *   - インフィル: 20% (三角形パターン)
 *   - レイヤー高さ: 0.2mm
 *   - プリント向き: 底板を下にして (サポート不要)
 * ============================================================================
 */

// ============================================================================
// パラメータ (URDF と一致)
// ============================================================================

// シャーシ寸法 [mm]
chassis_length = 300;       // X 方向 (前後) — URDF: 0.3m
chassis_width  = 200;       // Y 方向 (左右) — URDF: 0.2m
chassis_height = 100;       // Z 方向 (高さ) — URDF: 0.1m
wall_thickness = 3;         // 壁厚
bottom_thickness = 3;       // 底板厚

// 車輪 [mm] — URDF: wheel_radius=0.05, wheel_width=0.04
wheel_radius = 50;
wheel_width  = 40;
wheel_separation = 240;     // URDF: 0.24m
// 駆動輪は Y=±120mm (シャーシ幅 200mm の外側に出る)

// 駆動輪の X 位置 [mm] — URDF: -chassis_length/4
wheel_x_offset = -75;       // 後方

// モーター [mm] (DC ギアモーター想定)
motor_diameter = 25;
motor_length   = 60;
// モーター軸高さ: 車輪半径 = 50mm だが、シャーシ底面は wheel_radius - chassis_height/2 の位置
// URDF では origin z = -chassis_height/2 で車輪が付く → シャーシ底面とモーター軸が同じ高さ
motor_shaft_z = 0;          // シャーシ中心からの Z オフセット (底面から 50mm = 中心)

// キャスター [mm] — URDF: chassis_length/3 = +100mm
caster_x_offset = 100;      // 前方
caster_mount_diameter = 30;

// LiDAR マウント [mm]
lidar_mount_diameter = 70;  // RPLidar A1 の底面直径
lidar_mount_height = 15;
lidar_screw_diameter = 3;   // M3 ネジ穴
lidar_screw_spacing = 50;

// Pi 5 マウント [mm]
pi_width  = 85;
pi_length = 56;
pi_screw_diameter = 2.7;    // M2.5 ネジ穴
pi_mount_x = -30;
pi_mount_y = 0;
pi_standoff_height = 5;     // 基板と底板の間のスペーサー高さ
pi_standoff_diameter = 6;

// バッテリー収納 [mm]
battery_length = 130;
battery_width  = 45;
battery_height = 25;
battery_x_offset = 50;      // 前方 (重心調整)

// ヒートセットインサート穴 [mm]
insert_m3_hole = 4.0;       // M3 インサート用穴径
insert_m4_hole = 5.0;       // M4 インサート用穴径
insert_depth = 6;

// 刃モーターマウント [mm]
blade_motor_diameter = 28;
blade_motor_x = 100;        // 前方

// ============================================================================
// メインモジュール
// ============================================================================

module chassis_base() {
    // 底板 + 側壁の箱型 (上面開放)
    difference() {
        cube([chassis_length, chassis_width, chassis_height], center=true);

        // 内部くり抜き
        translate([0, 0, bottom_thickness])
            cube([
                chassis_length - wall_thickness * 2,
                chassis_width  - wall_thickness * 2,
                chassis_height
            ], center=true);
    }
}

module lidar_bridge() {
    // LiDAR マウントを支えるクロスバー (シャーシ上面)
    translate([0, 0, chassis_height/2 - bottom_thickness/2])
        cube([lidar_mount_diameter + 20, chassis_width - wall_thickness * 2, bottom_thickness], center=true);
}

module motor_cutout() {
    // モーターシャフト貫通穴 (左右の壁面)
    // 車輪は Y=±120mm でシャーシ幅 (Y=±100mm) の外側にあるため、
    // 壁面を貫通してモーターシャフトを通す
    for (side = [-1, 1]) {
        translate([wheel_x_offset, side * chassis_width/2, motor_shaft_z])
            rotate([90, 0, 0])
                cylinder(h=wall_thickness * 4, d=motor_diameter + 2, center=true, $fn=30);
    }
}

module motor_mount_holes() {
    // モーターマウント用 M4 ネジ穴 (左右の壁面、モーター軸の上下)
    for (side = [-1, 1]) {
        for (dz = [-15, 15]) {
            translate([wheel_x_offset, side * chassis_width/2, motor_shaft_z + dz])
                rotate([90, 0, 0])
                    cylinder(h=wall_thickness * 3, d=insert_m4_hole, center=true, $fn=20);
        }
    }
}

module caster_mount_hole() {
    // キャスター取り付け穴 (底板)
    translate([caster_x_offset, 0, -chassis_height/2])
        cylinder(h=bottom_thickness * 3, d=caster_mount_diameter, center=true, $fn=30);
}

module lidar_mount() {
    // LiDAR マウント台座 (クロスバーの上)
    translate([0, 0, chassis_height/2]) {
        difference() {
            cylinder(h=lidar_mount_height, d=lidar_mount_diameter + 10, $fn=40);

            // 中央穴 (配線通し)
            cylinder(h=lidar_mount_height * 3, d=20, center=true, $fn=20);

            // M3 ネジ穴 × 4
            for (angle = [0, 90, 180, 270]) {
                rotate([0, 0, angle])
                    translate([lidar_screw_spacing/2, 0, 0])
                        cylinder(h=lidar_mount_height * 3, d=lidar_screw_diameter, center=true, $fn=15);
            }
        }
    }
}

module pi_standoffs() {
    // Raspberry Pi 5 スタンドオフ (底板からの突起 + ネジ穴)
    // ショート防止のため基板を底板から浮かせる
    pi_hole_x = 58 / 2;
    pi_hole_y = 49 / 2;

    translate([pi_mount_x, pi_mount_y, -chassis_height/2 + bottom_thickness]) {
        for (dx = [-pi_hole_x, pi_hole_x]) {
            for (dy = [-pi_hole_y, pi_hole_y]) {
                translate([dx, dy, 0]) {
                    difference() {
                        // スタンドオフ柱
                        cylinder(h=pi_standoff_height, d=pi_standoff_diameter, $fn=20);
                        // M2.5 ネジ穴
                        cylinder(h=pi_standoff_height * 3, d=pi_screw_diameter, center=true, $fn=15);
                    }
                }
            }
        }
    }
}

module battery_retainer() {
    // バッテリー保持壁 (底板上に突起、バッテリーが動かないようにする)
    retainer_height = battery_height + 2;
    retainer_thickness = 2;

    translate([battery_x_offset, 0, -chassis_height/2 + bottom_thickness]) {
        // 前後の壁
        for (dx = [-(battery_length/2 + retainer_thickness/2), battery_length/2 + retainer_thickness/2]) {
            translate([dx, 0, retainer_height/2])
                cube([retainer_thickness, battery_width + 4, retainer_height], center=true);
        }
        // 左右の壁 (片側は開放 = バッテリー着脱用)
        translate([0, -(battery_width/2 + retainer_thickness/2), retainer_height/2])
            cube([battery_length, retainer_thickness, retainer_height], center=true);
    }
}

module blade_motor_hole() {
    // 刃モーター貫通穴 (底板)
    translate([blade_motor_x, 0, -chassis_height/2])
        cylinder(h=bottom_thickness * 3, d=blade_motor_diameter + 2, center=true, $fn=30);
}

module ventilation_slots() {
    // 通気口 (Pi 5 の排熱用、側面にルーバー状)
    slot_width = 2;
    slot_length = 30;
    num_slots = 5;

    for (side = [-1, 1]) {
        for (i = [0 : num_slots - 1]) {
            translate([
                pi_mount_x + (i - (num_slots-1)/2) * (slot_width + 3),
                side * chassis_width/2,
                15
            ])
                cube([slot_width, wall_thickness * 3, slot_length], center=true);
        }
    }
}

// ============================================================================
// 組み立て
// ============================================================================

module grass_chopper_chassis() {
    difference() {
        union() {
            chassis_base();
            lidar_bridge();
            lidar_mount();
            pi_standoffs();
            battery_retainer();
        }

        // 穴あけ
        motor_cutout();
        motor_mount_holes();
        caster_mount_hole();
        blade_motor_hole();
        ventilation_slots();
    }
}

// レンダリング
grass_chopper_chassis();
