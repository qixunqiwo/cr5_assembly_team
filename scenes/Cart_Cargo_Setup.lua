-- Cart_Cargo_Setup.lua
-- 在小车 CartA 和 CartB 上放置可见物料箱体
-- CartA 载 A 物料（灰色），CartB 载 B 物料（蓝橙）
-- 运行一次后禁用

sim = require('sim')

-- 在 cart 上创建一个货物箱体
local function addCargo(cartHandle, prefix, boxColor, stripeColor)
    if cartHandle == -1 then return nil end

    local cartPos = sim.getObjectPosition(cartHandle, -1)

    -- 主体箱体（与供料箱体尺寸接近）
    local body = sim.createPrimitiveShape(
        sim.primitiveshape_cuboid,
        {0.21, 0.15, 0.072}, 0)
    sim.setObjectAlias(body, prefix .. '_CargoBox')
    sim.setShapeColor(body, nil, sim.colorcomponent_ambient_diffuse, boxColor)
    -- 放在小车顶面（小车高约0.145m）
    sim.setObjectPosition(body, -1, {cartPos[1], cartPos[2], cartPos[3] + 0.10})
    sim.setObjectParent(body, cartHandle, true)

    -- 顶部条纹标记
    local stripe = sim.createPrimitiveShape(
        sim.primitiveshape_cuboid,
        {0.16, 0.03, 0.005}, 0)
    sim.setObjectAlias(stripe, prefix .. '_CargoStripe')
    sim.setShapeColor(stripe, nil, sim.colorcomponent_ambient_diffuse, stripeColor)
    sim.setObjectPosition(stripe, -1, {0, 0, 0.04})
    sim.setObjectParent(stripe, body, true)

    -- 小PCB板（放在箱体上方）
    local pcb = sim.createPrimitiveShape(
        sim.primitiveshape_cuboid,
        {0.14, 0.09, 0.005}, 0)
    sim.setObjectAlias(pcb, prefix .. '_CargoPCB')
    sim.setShapeColor(pcb, nil, sim.colorcomponent_ambient_diffuse, stripeColor)
    sim.setObjectPosition(pcb, -1, {0, 0.02, 0.043})
    sim.setObjectParent(pcb, body, true)

    return body
end

function sysCall_init()
    print('===== Cart Cargo Setup =====')

    local CartA = sim.getObject('/CartA')
    local CartB = sim.getObject('/CartB')

    if CartA == -1 or CartB == -1 then
        print('[ERROR] CartA or CartB not found')
        return
    end

    -- CartA: 灰色箱体 + 绿色条纹（A物料配色）
    local cargoA = addCargo(CartA, 'A', {0.62, 0.62, 0.62}, {0.0, 0.45, 0.18})
    if cargoA then
        -- 初始隐藏（CartA 在等待位）
        local objs = sim.getObjectsInTree(cargoA, sim.handle_all, 0)
        for _, h in ipairs(objs) do
            sim.setObjectInt32Param(h, sim.objintparam_visibility_layer, 0)
        end
        print('[OK] CartA cargo: gray box + green stripe')
    end

    -- CartB: 蓝色箱体 + 橙色条纹（B物料配色）
    local cargoB = addCargo(CartB, 'B', {0.20, 0.25, 0.60}, {0.95, 0.40, 0.10})
    if cargoB then
        local objs = sim.getObjectsInTree(cargoB, sim.handle_all, 0)
        for _, h in ipairs(objs) do
            sim.setObjectInt32Param(h, sim.objintparam_visibility_layer, 0)
        end
        print('[OK] CartB cargo: blue box + orange stripe')
    end

    print('===== Setup done. Cargo visibility controlled by cart position. =====')
    print('Run Cart_Cargo_Controller.lua to auto show/hide during orders.')
end
