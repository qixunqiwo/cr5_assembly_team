-- Cart_Cargo_Full.lua
-- 一体化脚本：创建货物 + 自动显隐
-- 直接挂载为长期运行脚本即可，首次运行自动创建货物

sim = require('sim')

local CartA, CartB
local A_Supply, B_Supply
local cargoRoots = {}  -- {cartHandle = cargoRootHandle}
local setupDone = false

local function copyPartsToCart(sourcePath, cartHandle, prefix)
    local source = sim.getObject(sourcePath)
    if source == -1 then return nil end
    local objs = sim.getObjectsInTree(source, sim.handle_all, 0)
    local copy = sim.copyPasteObjects(objs, 0)
    local root = nil
    for _, h in ipairs(copy) do
        if sim.getObjectParent(h) == -1 then root = h; break end
    end
    if not root then return nil end
    sim.setObjectAlias(root, prefix)
    -- 重命名所有子对象避免别名冲突
    for _, h in ipairs(copy) do
        if h ~= root then
            local n = ''
            pcall(function() n = sim.getObjectAlias(h, 0) or '' end)
            if n ~= '' then
                pcall(function() sim.setObjectAlias(h, n .. '_cargo') end)
            end
        end
    end
    sim.setObjectParent(root, cartHandle, true)
    -- 初始隐藏
    for _, h in ipairs(copy) do
        sim.setObjectInt32Param(h, sim.objintparam_visibility_layer, 0)
    end
    return root
end

local function setVisible(root, visible)
    if root == -1 then return end
    local objs = sim.getObjectsInTree(root, sim.handle_all, 0)
    for _, h in ipairs(objs) do
        sim.setObjectInt32Param(h, sim.objintparam_visibility_layer, visible and 1 or 0)
    end
end

local function isAt(cart, target)
    if cart == -1 or target == -1 then return false end
    local cp = sim.getObjectPosition(cart, -1)
    local tp = sim.getObjectPosition(target, -1)
    return math.sqrt((cp[1]-tp[1])^2 + (cp[2]-tp[2])^2 + (cp[3]-tp[3])^2) < 0.015
end

function sysCall_init()
    CartA = sim.getObject('/CartA')
    CartB = sim.getObject('/CartB')
    A_Supply = sim.getObject('/CartA_SupplyPose')
    B_Supply = sim.getObject('/CartB_SupplyPose')

    if CartA == -1 or CartB == -1 then
        print('[Cargo] Carts not found')
        return
    end

    -- 检查是否已创建
    local testA = sim.getObject('/CartA/A_CargoParts')
    local testB = sim.getObject('/CartB/B_CargoParts')

    if testA == -1 then
        print('[Cargo] Creating A parts on CartA...')
        local a1 = copyPartsToCart('/FiveCR5A_Cell/Parts/Box_Blank', CartA, 'A_CargoParts')
        if a1 then
            sim.setObjectPosition(a1, CartA, {0, 0, 0.10})
            local a2 = copyPartsToCart('/FiveCR5A_Cell/Parts/PCB_Supply', CartA, 'A_CargoPCB')
            if a2 ~= -1 then sim.setObjectPosition(a2, CartA, {0, 0, 0.143}) end
            local a3 = copyPartsToCart('/FiveCR5A_Cell/Parts/Control_Module_Supply', CartA, 'A_CargoModule')
            if a3 ~= -1 then sim.setObjectPosition(a3, CartA, {0.025, 0.025, 0.15}) end
            local a4 = copyPartsToCart('/FiveCR5A_Cell/Parts/Terminal_Block_Supply', CartA, 'A_CargoTerminal')
            if a4 ~= -1 then sim.setObjectPosition(a4, CartA, {0.045, -0.035, 0.14}) end
            print('[Cargo] CartA parts created')
        end
    else
        print('[Cargo] CartA parts already exist')
    end

    if testB == -1 then
        print('[Cargo] Creating B parts on CartB...')
        local b1 = copyPartsToCart('/FiveCR5A_Cell/PartsB/Box_Blank_B', CartB, 'B_CargoParts')
        if b1 then
            sim.setObjectPosition(b1, CartB, {0, 0, 0.10})
            local b2 = copyPartsToCart('/FiveCR5A_Cell/PartsB/PCB_Supply_B', CartB, 'B_CargoPCB')
            if b2 ~= -1 then sim.setObjectPosition(b2, CartB, {0, 0, 0.143}) end
            local b3 = copyPartsToCart('/FiveCR5A_Cell/PartsB/Control_Module_Supply_B', CartB, 'B_CargoModule')
            if b3 ~= -1 then sim.setObjectPosition(b3, CartB, {0.025, 0.025, 0.15}) end
            local b4 = copyPartsToCart('/FiveCR5A_Cell/PartsB/Terminal_Block_Supply_B', CartB, 'B_CargoTerminal')
            if b4 ~= -1 then sim.setObjectPosition(b4, CartB, {0.045, -0.035, 0.14}) end
            print('[Cargo] CartB parts created')
        end
    else
        print('[Cargo] CartB parts already exist')
    end

    print('[Cargo] Ready')
end

function sysCall_actuation()
    if CartA == -1 or CartB == -1 then return end

    -- 刷新 cargo 引用
    local aCargo = sim.getObject('/CartA/A_CargoParts')
    local bCargo = sim.getObject('/CartB/B_CargoParts')

    -- CartA cargo: 等待位显示，供料位隐藏
    if aCargo ~= -1 then
        setVisible(aCargo, not isAt(CartA, A_Supply))
        -- 同步子零件
        for _, prefix in ipairs({'A_CargoPCB', 'A_CargoModule', 'A_CargoTerminal'}) do
            local h = sim.getObject('/CartA/' .. prefix)
            if h ~= -1 then setVisible(h, not isAt(CartA, A_Supply)) end
        end
    end

    -- CartB cargo: 等待位显示，供料位隐藏
    if bCargo ~= -1 then
        setVisible(bCargo, not isAt(CartB, B_Supply))
        for _, prefix in ipairs({'B_CargoPCB', 'B_CargoModule', 'B_CargoTerminal'}) do
            local h = sim.getObject('/CartB/' .. prefix)
            if h ~= -1 then setVisible(h, not isAt(CartB, B_Supply)) end
        end
    end
end
