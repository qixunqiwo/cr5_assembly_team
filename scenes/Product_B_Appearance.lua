-- Product_B_Appearance.lua
-- B产品外观大幅修改 - 不影响夹取和装配
-- 改动范围：外壁纹理/颜色、底部扩展、角部加强
-- 不动：顶部开口、内部空间、夹持面宽度
-- 运行一次后禁用

sim = require('sim')

function sysCall_init()
    print('===== B产品外观修改 =====')

    local PartsB = sim.getObject('/FiveCR5A_Cell/PartsB')
    if PartsB == -1 then
        print('[ERROR] PartsB not found.')
        return
    end

    local boxB = sim.getObject('/FiveCR5A_Cell/PartsB/Box_Blank_B')
    if boxB == -1 then
        print('[ERROR] Box_Blank_B not found.')
        return
    end

    local boxPos = sim.getObjectPosition(boxB, -1)
    -- 箱体半尺寸（60%缩放）: L=0.21, W=0.15, H=0.072
    local HL = 0.105  -- 半长 X
    local HW = 0.075  -- 半宽 Y
    local HZ = 0.036  -- 半高 Z

    -- ============================================
    -- 1. 箱体配色：不同面板用不同颜色
    -- ============================================
    local objs = sim.getObjectsInTree(boxB, sim.handle_all, 0)
    for i = 1, #objs do
        local n = ''
        pcall(function() n = sim.getObjectAlias(objs[i], 0) or '' end)
        if string.find(n, 'Bottom') then
            sim.setShapeColor(objs[i], nil, sim.colorcomponent_ambient_diffuse, {0.15, 0.15, 0.15})
        elseif string.find(n, 'Front_Wall') then
            sim.setShapeColor(objs[i], nil, sim.colorcomponent_ambient_diffuse, {0.95, 0.40, 0.10})
        elseif string.find(n, 'Back_Wall') then
            sim.setShapeColor(objs[i], nil, sim.colorcomponent_ambient_diffuse, {0.08, 0.15, 0.50})
        elseif string.find(n, 'Left_Wall') then
            sim.setShapeColor(objs[i], nil, sim.colorcomponent_ambient_diffuse, {0.20, 0.25, 0.60})
        elseif string.find(n, 'Right_Wall') then
            sim.setShapeColor(objs[i], nil, sim.colorcomponent_ambient_diffuse, {0.20, 0.25, 0.60})
        elseif string.find(n, 'Post') then
            sim.setShapeColor(objs[i], nil, sim.colorcomponent_ambient_diffuse, {1.0, 0.84, 0.0})
        elseif string.find(n, 'EndCover') then
            sim.setShapeColor(objs[i], nil, sim.colorcomponent_ambient_diffuse, {0.95, 0.35, 0.05})
        end
    end
    print('[OK] panels colored')

    -- ============================================
    -- 2. 底部扩展法兰（比箱体略宽，不挡夹持面）
    -- ============================================
    local flange = sim.createPrimitiveShape(
        sim.primitiveshape_cuboid,
        {0.23, 0.17, 0.006}, 0
    )
    sim.setObjectAlias(flange, 'B_Base_Flange')
    sim.setShapeColor(flange, nil, sim.colorcomponent_ambient_diffuse, {0.12, 0.12, 0.14})
    sim.setObjectPosition(flange, -1, {boxPos[1], boxPos[2], boxPos[3] - 0.039})
    sim.setObjectParent(flange, boxB, true)
    print('[OK] bottom flange added')

    -- ============================================
    -- 3. 外壁竖条纹（散热片效果，不增加夹持面宽度）
    -- ============================================
    local function addRib(name, x, y, z, sx, sy, sz)
        local rib = sim.createPrimitiveShape(sim.primitiveshape_cuboid, {sx, sy, sz}, 0)
        sim.setObjectAlias(rib, name)
        sim.setShapeColor(rib, nil, sim.colorcomponent_ambient_diffuse, {0.08, 0.12, 0.45})
        sim.setObjectPosition(rib, -1, {x, y, z})
        sim.setObjectParent(rib, boxB, true)
    end

    -- 左侧外壁竖条纹（3条）
    local lx = boxPos[1] - HL - 0.004  -- 贴在左壁外侧
    for j = 1, 3 do
        local ry = boxPos[2] + (j - 2) * 0.04
        addRib('B_Rib_L'..j, lx, ry, boxPos[3], 0.004, 0.015, 0.055)
    end

    -- 右侧外壁竖条纹（3条）
    local rx = boxPos[1] + HL + 0.004
    for j = 1, 3 do
        local ry = boxPos[2] + (j - 2) * 0.04
        addRib('B_Rib_R'..j, rx, ry, boxPos[3], 0.004, 0.015, 0.055)
    end

    -- 后壁横条纹（2条）
    local by = boxPos[2] + HW + 0.004
    for j = 1, 2 do
        local rxx = boxPos[1] + (j - 1.5) * 0.07
        addRib('B_Rib_B'..j, rxx, by, boxPos[3], 0.025, 0.004, 0.050)
    end
    print('[OK] external ribs added')

    -- ============================================
    -- 4. 四角底部加强筋（三角柱）
    -- ============================================
    local corners = {
        {boxPos[1] - HL, boxPos[2] - HW},
        {boxPos[1] + HL, boxPos[2] - HW},
        {boxPos[1] - HL, boxPos[2] + HW},
        {boxPos[1] + HL, boxPos[2] + HW},
    }
    for j, c in ipairs(corners) do
        local gusset = sim.createPrimitiveShape(
            sim.primitiveshape_cuboid,
            {0.016, 0.016, 0.025}, 0
        )
        sim.setObjectAlias(gusset, 'B_Gusset_' .. j)
        sim.setShapeColor(gusset, nil, sim.colorcomponent_ambient_diffuse, {0.90, 0.45, 0.10})
        sim.setObjectPosition(gusset, -1, {c[1], c[2], boxPos[3] - 0.042})
        sim.setObjectParent(gusset, boxB, true)
    end
    print('[OK] corner gussets added')

    -- ============================================
    -- 5. 前壁B标记（小橙色方块，不挡夹持）
    -- ============================================
    local frontY = boxPos[2] - HW - 0.004
    local marker = sim.createPrimitiveShape(
        sim.primitiveshape_cuboid,
        {0.03, 0.004, 0.02}, 0
    )
    sim.setObjectAlias(marker, 'B_Marker')
    sim.setShapeColor(marker, nil, sim.colorcomponent_ambient_diffuse, {1.0, 0.90, 0.05})
    sim.setObjectPosition(marker, -1, {boxPos[1] - 0.04, frontY, boxPos[3] + 0.015})
    sim.setObjectParent(marker, boxB, true)

    local marker2 = sim.createPrimitiveShape(
        sim.primitiveshape_cuboid,
        {0.03, 0.004, 0.02}, 0
    )
    sim.setObjectAlias(marker2, 'B_Marker2')
    sim.setShapeColor(marker2, nil, sim.colorcomponent_ambient_diffuse, {1.0, 0.90, 0.05})
    sim.setObjectPosition(marker2, -1, {boxPos[1] + 0.04, frontY, boxPos[3] + 0.015})
    sim.setObjectParent(marker2, boxB, true)
    print('[OK] front markers added')

    -- ============================================
    -- 6. PCB: 深紫板+银芯片+金孔
    -- ============================================
    local function recolorPCB(root)
        if root == -1 then return end
        local objs = sim.getObjectsInTree(root, sim.handle_all, 0)
        for i = 1, #objs do
            local n = ''
            pcall(function() n = sim.getObjectAlias(objs[i], 0) or '' end)
            if string.find(n, 'Board') then
                sim.setShapeColor(objs[i], nil, sim.colorcomponent_ambient_diffuse, {0.35, 0.05, 0.35})
            elseif string.find(n, 'Chip') then
                sim.setShapeColor(objs[i], nil, sim.colorcomponent_ambient_diffuse, {0.85, 0.85, 0.90})
            elseif string.find(n, 'Hole') then
                sim.setShapeColor(objs[i], nil, sim.colorcomponent_ambient_diffuse, {1.0, 0.75, 0.15})
            elseif string.find(n, 'Connector') then
                sim.setShapeColor(objs[i], nil, sim.colorcomponent_ambient_diffuse, {1.0, 0.90, 0.10})
            end
        end
    end
    recolorPCB(sim.getObject('/FiveCR5A_Cell/PartsB/PCB_Supply_B'))
    recolorPCB(sim.getObject('/FiveCR5A_Cell/PartsB/Assembly_ControlBox_Product_B'))
    recolorPCB(sim.getObject('/FiveCR5A_Cell/PartsB/Inspection_ControlBox_Product_B'))
    print('[OK] PCB: purple + silver chips')

    -- ============================================
    -- 7. 模块: 亮橙红+白标签
    -- ============================================
    local function recolorModule(root)
        if root == -1 then return end
        local objs = sim.getObjectsInTree(root, sim.handle_all, 0)
        for i = 1, #objs do
            local n = ''
            pcall(function() n = sim.getObjectAlias(objs[i], 0) or '' end)
            if string.find(n, 'Body') then
                sim.setShapeColor(objs[i], nil, sim.colorcomponent_ambient_diffuse, {0.95, 0.25, 0.15})
            elseif string.find(n, 'Label') then
                sim.setShapeColor(objs[i], nil, sim.colorcomponent_ambient_diffuse, {1.0, 1.0, 1.0})
            end
        end
    end
    recolorModule(sim.getObject('/FiveCR5A_Cell/PartsB/Control_Module_Supply_B'))
    recolorModule(sim.getObject('/FiveCR5A_Cell/PartsB/Assembly_ControlBox_Product_B'))
    recolorModule(sim.getObject('/FiveCR5A_Cell/PartsB/Inspection_ControlBox_Product_B'))
    print('[OK] module: orange-red + white label')

    -- ============================================
    -- 8. 端子排: 翠绿+银螺钉
    -- ============================================
    local function recolorTerminal(root)
        if root == -1 then return end
        local objs = sim.getObjectsInTree(root, sim.handle_all, 0)
        for i = 1, #objs do
            local n = ''
            pcall(function() n = sim.getObjectAlias(objs[i], 0) or '' end)
            if string.find(n, 'Body') then
                sim.setShapeColor(objs[i], nil, sim.colorcomponent_ambient_diffuse, {0.10, 0.70, 0.25})
            elseif string.find(n, 'Slot') then
                sim.setShapeColor(objs[i], nil, sim.colorcomponent_ambient_diffuse, {0.02, 0.15, 0.05})
            elseif string.find(n, 'Screw') then
                sim.setShapeColor(objs[i], nil, sim.colorcomponent_ambient_diffuse, {0.90, 0.90, 0.92})
            end
        end
    end
    recolorTerminal(sim.getObject('/FiveCR5A_Cell/PartsB/Terminal_Block_Supply_B'))
    recolorTerminal(sim.getObject('/FiveCR5A_Cell/PartsB/Assembly_ControlBox_Product_B'))
    recolorTerminal(sim.getObject('/FiveCR5A_Cell/PartsB/Inspection_ControlBox_Product_B'))
    print('[OK] terminal: green + silver screws')

    print('===== B产品外观修改完成 =====')
    print('')
    print('A vs B 对比:')
    print('  A: 全灰箱体 + 绿PCB + 深灰模块 + 黄端子')
    print('  B: 蓝橙面板箱体 + 底部法兰 + 外壁散热条纹')
    print('     + 四角橙色加强筋 + 前壁亮黄标记')
    print('     + 紫PCB银芯片 + 橙红模块白标签 + 绿端子银螺钉')
    print('')
    print('顶部开口不变，夹持面不变，装配不受影响。')
end
