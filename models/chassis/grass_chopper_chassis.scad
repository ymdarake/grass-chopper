/**
 * ============================================================================
 * grass-chopper シャーシ (OpenSCAD パラメトリックモデル)
 * ============================================================================
 * URDF (robot_description.urdf.xacro) の寸法と一致させた
 * 3D プリント用シャーシモデル。
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
 * ============================================================================
 */

// ============================================================================
// パラメータ (URDF と一致)
// ============================================================================

// シャーシ寸法 [mm]
chassis_length = 300;       // X 方向 (前後)
chassis_width  = 200;       // Y 方向 (左右)
chassis_height = 80;        // Z 方向 (高さ)
wall_thickness = 3;         // 壁厚
bottom_thickness = 3;       // 底板厚

// 車輪 [mm]
wheel_radius = 50;          // 車輪半径
wheel_width  = 40;          // 車輪幅
wheel_separation = 240;     // 車輪間距離 (中心間)
wheel_y_offset = wheel_separation / 2;

// モーター [mm] (DC ギアモーター想定)
motor_diameter = 25;        // モーター直径
motor_length   = 60;        // モーター長さ (ギアボックス含む)
motor_shaft_height = 30;    // シャーシ底面からモーター軸中心までの高さ
motor_mount_thickness = 4;  // モーターマウント厚

// キャスター [mm]
caster_radius = 25;         // キャスター半径
caster_x_offset = -120;     // シャーシ中心からの X オフセット (後方)
caster_mount_diameter = 30; // 取り付け穴径

// LiDAR マウント [mm]
lidar_mount_diameter = 70;  // RPLidar A1 の底面直径
lidar_mount_height = 15;    // マウント高さ
lidar_screw_diameter = 3;   // M3 ネジ穴
lidar_screw_spacing = 50;   // ネジ穴間隔

// Pi 5 マウント [mm]
pi_width  = 85;             // Pi 5 基板幅
pi_length = 56;             // Pi 5 基板長さ
pi_screw_diameter = 2.7;    // M2.5 ネジ穴
pi_mount_x = -30;           // シャーシ中心からの X オフセット
pi_mount_y = 0;

// バッテリー収納 [mm]
battery_length = 130;       // 3S LiPo
battery_width  = 45;
battery_height = 25;
battery_x_offset = 50;      // シャーシ前方 (重心調整)

// ヒートセットインサート穴 [mm]
insert_m3_hole = 4.0;       // M3 インサート用穴径
insert_m4_hole = 5.0;       // M4 インサート用穴径
insert_depth = 6;           // インサート埋め込み深さ

// 刃モーターマウント [mm]
blade_motor_diameter = 28;  // BLDC モーター直径
blade_motor_x = 100;        // シャーシ前方

// ============================================================================
// メインモジュール
// ============================================================================

module chassis_base() {
    // 底板 + 側壁の箱型
    difference() {
        // 外形
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

module motor_cutout() {
    // モーター貫通穴 (左右)
    for (side = [-1, 1]) {
        translate([0, side * wheel_y_offset, motor_shaft_height - chassis_height/2])
            rotate([0, 90, 0])
                rotate([0, 0, 90])
                    cylinder(h=wall_thickness * 3, d=motor_diameter + 2, center=true);
    }
}

module motor_mount_holes() {
    // モーターマウント用 M4 ネジ穴 (左右各 2 穴)
    for (side = [-1, 1]) {
        for (dx = [-15, 15]) {
            translate([dx, side * (chassis_width/2), motor_shaft_height - chassis_height/2])
                rotate([90, 0, 0])
                    cylinder(h=wall_thickness * 2, d=insert_m4_hole, center=true, $fn=20);
        }
    }
}

module caster_mount_hole() {
    // キャスター取り付け穴
    translate([caster_x_offset, 0, -chassis_height/2])
        cylinder(h=bottom_thickness * 3, d=caster_mount_diameter, center=true, $fn=30);
}

module lidar_mount() {
    // LiDAR マウント台座 (シャーシ上面)
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

module pi_mount_holes() {
    // Raspberry Pi 5 マウント穴 (M2.5 × 4)
    // Pi 5 のネジ穴位置: 49mm × 58mm
    pi_hole_x = 58 / 2;
    pi_hole_y = 49 / 2;

    translate([pi_mount_x, pi_mount_y, -chassis_height/2]) {
        for (dx = [-pi_hole_x, pi_hole_x]) {
            for (dy = [-pi_hole_y, pi_hole_y]) {
                translate([dx, dy, 0])
                    cylinder(h=bottom_thickness * 3, d=pi_screw_diameter, center=true, $fn=15);
            }
        }
    }
}

module battery_compartment() {
    // バッテリー収納エリア (底板にくぼみ)
    translate([battery_x_offset, 0, -chassis_height/2 + bottom_thickness + battery_height/2])
        cube([battery_length + 2, battery_width + 2, battery_height], center=true);
}

module blade_motor_hole() {
    // 刃モーター貫通穴 (底面)
    translate([blade_motor_x, 0, -chassis_height/2])
        cylinder(h=bottom_thickness * 3, d=blade_motor_diameter + 2, center=true, $fn=30);
}

module ventilation_slots() {
    // 通気口 (Pi 5 の排熱用、側面に配置)
    slot_width = 2;
    slot_length = 30;
    num_slots = 5;

    for (side = [-1, 1]) {
        for (i = [0 : num_slots - 1]) {
            translate([
                pi_mount_x + (i - (num_slots-1)/2) * (slot_width + 3),
                side * chassis_width/2,
                20 - chassis_height/2
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
            lidar_mount();
        }

        // 穴あけ
        motor_cutout();
        motor_mount_holes();
        caster_mount_hole();
        pi_mount_holes();
        battery_compartment();
        blade_motor_hole();
        ventilation_slots();
    }
}

// レンダリング
grass_chopper_chassis();
