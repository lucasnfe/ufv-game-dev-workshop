pico-8 cartridge // http://www.pico-8.com
version 16
__lua__
----------------------
--- main program ---
----------------------

g_const = 0.098

function init_obj(s, pos)
	local new_obj = {}
	new_obj.pos = pos
	new_obj.s = s
	return new_obj
end

function _init()
	player = init_obj(1,v_init(50,50))

	-- add physics capabilities to the player
	phy_init_obj(player, 1, v_init(2,2))

	camera_pos = v_init(0,0)
	camera_offset = v_init(60,60)
end

function _update()
	-- Running mechanics
	if(btn(0) == true) then
		 -- player.pos.x = player.pos.x - player.speed.x
		 player.vel.x = player.speed.x
		 sfx(0)
	end

	if(btn(1) == true) then
		-- player.pos.x = player.pos.x + player.speed.x
		player.vel.x = player.speed.x
		sfx(0)
	end

	phy_update(player)

	-- Jump mechanics
	if(btn(2) == true and player.is_on_ground) then
		player.vel.y = -player.speed.y
		player.is_on_ground = false
	end

	camera_pos = v_sub(player.pos, camera_offset)
	camera(camera_pos.x, camera_pos.y)
end

function draw_obj(obj)
	spr(obj.s, obj.pos.x, obj.pos.y)
end

function _draw()
    cls()

    -- draw map
    map(0,0,0,0,128,32)

    -- draw player
    draw_obj(player)
end
-->8
----------------------
--- vector library ---
----------------------

function v_init(x, y)
	local v = {}
	v.x = x
	v.y = y
	return v
end

function v_add(v, u)
	return v_init(v.x+u.x, v.y+u.y)
end

function v_sub(v,u)
 return v_init(v.x-u.x, v.y-u.y)
end

function v_mul(v, s)
	return v_init(v.x*s, v.y*s)
end

function v_div(v, s)
	if(s == 0) then
		return nil
	end

	return v_init(v.x/s, v.y/s)
end

function v_mag(v)
	return sqrt(v.x*v.x + v.y*v.y)
end

function v_norm(v)
	return v_div(v,v_mag(v))
end

function v_dot(v, u)
	return v.x*u.x + v.y*u.y
end

function v_cross(v, u)
	return v.x*u.y - v.y*u.x
end

-- function v_angle(v1,v2)
-- 	return atan2(v_cross(v1,v2), v_dot(v1,v2))
-- end
--
-- function v_str(v1)
-- 	return "("..v1.x..","..v1.y..")"
-- end
--
-- function v_print(v1)
-- 	print(v_str(v1))
-- end

-->8
-------------------------------------
---------- physics library ----------
-------------------------------------

function phy_is_solid(map_pos)
	-- Flag 0 means solid sprites
	local m_sprite = mget(map_pos.x, map_pos.y)
	return fget(m_sprite, 0)
end

function phy_screen_to_map(pos)
	return v_init(flr(pos.x/8), flr(pos.y/8))
end

function phy_map_to_screen(pos)
	return v_mul(pos, 8)
end

function phy_aabb_collide(pos_a, a, pos_b, b)
	if(pos_a.x + a.max.x < pos_b.x + b.min.x or
	   pos_b.x + b.max.x < pos_a.x + a.min.x) then
		return false
	end

	if(pos_a.y + a.max.y < pos_b.y + b.min.y or
	   pos_b.y + b.max.y < pos_a.y + a.min.y) then
		return false
	end

	return true
end

function phy_aabb_init(x,y,w,h)
	local aabb = {}
	aabb.w = w
	aabb.h = h
	aabb.min = v_init(x,y)
	aabb.max = v_init(x+w,y+h)
	return aabb
end

function phy_init_obj(game_obj, mass, speed)
	if(game_obj == nil) then
		return nil
	end

	game_obj.acc = v_init(0,0)
	game_obj.vel = v_init(0,0)
	game_obj.mass = mass or 1
	game_obj.speed = speed or v_init(1,1)
	game_obj.is_on_ground = false
	return game_obj
end

function phy_apply_friction(phy_obj, grd_fric)
	if(abs(phy_obj.vel.x) > 0) then
		local n_vel_x = v_norm(phy_obj.vel).x * -1
		local fric = v_init(n_vel_x * grd_fric, 0)
		phy_apply_force(phy_obj, fric)
	end
end

-- collision detection
function phy_detect_horz_collision(phy_obj)

end

function phy_detect_vert_collision(phy_obj)
	-- project pos in the next frame
	local proj_pos_y = phy_obj.pos.y + phy_obj.vel.y

	-- calc corner points
	local col_point1 = v_init(phy_obj.pos.x, phy_obj.pos.y)
	local col_point2 = v_add(phy_obj.pos, v_init(8,0)) -- 8 is the size of the sprite

	local dir_y = sgn(phy_obj.vel.y)
	if(dir_y > 0) then
		col_point1 = v_add(phy_obj.pos, v_init(0,8))
		col_point2 = v_add(phy_obj.pos, v_init(8,8))
	end

	-- Find which cell the obj is in at this moment
	map_col_cell1 = phy_screen_to_map(col_point1)
	map_col_cell2 = phy_screen_to_map(col_point2)

	if(phy_is_solid(map_col_cell1) or
	   phy_is_solid(map_col_cell2)) then
		   -- Reset velocity and push object back to surface
		   phy_obj.vel.y = 0
		   phy_obj.pos.y = phy_map_to_screen(map_col_cell1).y - dir_y*8

		   -- Obj is falling and touched the ground
		   if(dir_y > 0) then
		   	  phy_obj.is_on_ground = true
		   end
	end
end

-- basic movement
function phy_apply_force(phy_obj, f)
	local f = v_div(f, phy_obj.mass)
	phy_obj.acc = v_add(phy_obj.acc, f)
end

function phy_update(phy_obj)
	-- apply gravity
	local g_force = v_init(0, g_const)
	phy_apply_force(phy_obj, v_mul(g_force, phy_obj.mass))

	phy_obj.vel = v_add(phy_obj.vel, phy_obj.acc)

	-- detect horizontal collision
	if(abs(phy_obj.vel.x) > 0) then
		phy_detect_horz_collision(phy_obj)
		phy_apply_friction(phy_obj, 0.5)
	end

	-- detect vertical collision
	if(abs(phy_obj.vel.y) > 0) then
		phy_detect_vert_collision(phy_obj)
	end

	phy_obj.pos = v_add(phy_obj.pos, phy_obj.vel)

	phy_obj.acc = v_init(0,0)
end

-------------------------------------
----------- ai library --------------
-------------------------------------

function pf_pop(queue)
	if(#queue > 0) then
		local f_elem = queue[1]
		del(queue, f_elem)
		return f_elem
	end

	return nil
end



__gfx__
00000000000000005555555544444444cccccccc0000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
00000000033333304444444444444444cccccccc0000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
00700700036446304544444444444444cccccccc0000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
00077000034444304454454444444444cccccccc0000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
00077000034444304444445444444444cccccccc0000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
00700700033333304444444444444444cccccccc0000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
00000000003003004454444444444444cccccccc0000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
00000000000000004445444444444444cccccccc0000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
__gff__
0001010000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
0000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
__map__
0404040404040404040404040404040404040404040404040404040404040404000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
0404040404040404040404040404040404040404040404040404040404040404000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
0404040404040404040404040404040404040404040404040404040404040404000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
0404040404040404040404040404040404040404040404040404040404040404000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
0404040404040404040404040404040404040404040404040404040404040404000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
0404040404040404040404040404040404040404040404040404040404040404000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
0404040404040404040404040404040404040404040404040404040404040404000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
0404040404040404040404040404040404040404040404040404040404040404000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
0404040404040404040404040404040404040404040404040404040404040404000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
0404040404040404040404040404040404040404040404040404040404040404000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
0202020202020202020202020202020202020202020202020404040202020202000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
0303030303030303030303030303030303030303030303030404040303030303000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
0303030303030303030303030303030303030303030303030404040303030303000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
__sfx__
010200002655029552265520000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
