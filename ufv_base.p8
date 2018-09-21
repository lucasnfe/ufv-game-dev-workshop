pico-8 cartridge // http://www.pico-8.com
version 8
__lua__
-- lec12: visual effects
-- lucas n. ferreira

-------------------------------------
----------- math library ----------
-------------------------------------

m_infinity = 32767

function m_project_point(seg_s, seg_e, p)
	local a = v_sub(p, seg_s)
	local b = v_sub(seg_e, seg_s)

	local theta = v_angle(a, b)

	b = v_norm(b)
	b = v_mul(b, v_mag(a) * cos(theta))

	return v_add(seg_s, b)
end

-------------------------------------
----------- vector library ----------
-------------------------------------

function v_init(x,y)
	local v1 = {}
	v1.x = x
	v1.y = y
	return v1
end

function v_add(v1,v2)
	return v_init(v1.x + v2.x, v1.y + v2.y)
end

function v_sub(v1,v2)
	return v_init(v1.x - v2.x, v1.y - v2.y)
end

function v_div(v1, s)
	if(s != 0) then
		return v_init(v1.x / s, v1.y / s)
	end

	return v1
end

function v_mul(v1, s)
	return v_init(v1.x * s, v1.y * s)
end

function v_mag(v1)
	local mag = sqrt((v1.x * v1.x) + (v1.y * v1.y))
	if(mag < 0 or mag > m_infinity) then
		return m_infinity
	end
	return mag
end

function v_dist(v1, v2)
	local dist = sqrt((v1.x-v2.x)^2 + (v1.y-v2.y)^2)
	if(dist < 0 or dist > m_infinity) then
		return m_infinity
	end
	return dist
end

function v_clamp(v1, max_mag)
	if(v_mag(v1) > max_mag) then
		v1 = v_norm(v1)
		v1 = v_mul(v1, max_mag)
	end
	return v1
end

function v_norm(v1)
	if(v_mag(v1) != 0) then
		return v_div(v1, v_mag(v1))
	end
	return nil
end

function v_str(v1)
	return "("..v1.x..","..v1.y..")"
end

function v_print(v1)
	print(v_str(v1))
end

function v_dot(v1,v2)
	return v1.x*v2.x+v1.y*v2.y
end

function v_cross(v1,v2)
	return v1.x*v2.x-v1.y*v2.y
end

function v_angle(v1,v2)
	return atan2(v_cross(v1,v2), v_dot(v1,v2))
end
-->8

-------------------------------------
----------- graphics library ----------
-------------------------------------

fades={
	{1,1,1,1,0,0,0,0},
	{2,2,2,1,1,0,0,0},
	{3,3,4,5,2,1,1,0},
	{4,4,2,2,1,1,1,0},
	{5,5,2,2,1,1,1,0},
	{6,6,13,5,2,1,1,0},
	{7,7,6,13,5,2,1,0},
	{8,8,9,4,5,2,1,0},
	{9,9,4,5,2,1,1,0},
	{10,15,9,4,5,2,1,0},
	{11,11,3,4,5,2,1,0},
	{12,12,13,5,5,2,1,0},
	{13,13,5,5,2,1,1,0},
	{14,9,9,4,5,2,1,0},
	{15,14,9,4,5,2,1,0}
}

function fade(fa)
	fa=max(min(1,fa),0)
	local fn=8
	local pn=15
	local fc=1/fn
	local fi=flr(fa/fc)+1
	for n=1,pn do
		pal(n,fades[n][fi],0)
	end
end

-------------------------------------
----------- debug library -----------
-------------------------------------

deb_mode_on = false
deb_states = {"normal","step", "slowmo"}
deb_curt_state = 1
deb_frame_timer = 0

deb_log_buffer = {}
deb_text_buffer = {}
deb_rect_buffer = {}

function deb_start()
	deb_frame_timer = 0
	deb_mode_on = not deb_mode_on
end

function deb_log(msg)
	add(deb_log_buffer, msg)
end

function deb_text(msg, x, y)
	local text_entry = {}
	text_entry.msg = msg
	text_entry.x = x
	text_entry.y = y
	add(deb_text_buffer, text_entry)
end

function deb_path(path)
	if(path != nil and #path > 2) then
		local s = phy_map_to_screen(path[1])
		circfill(s.x, s.y, 2, 1)

		for i=1, #path-1 do
			local a = phy_map_to_screen(path[i])
			local b = phy_map_to_screen(path[i+1])
			line(a.x, a.y, b.x, b.y, 7)
		end

		local g = phy_map_to_screen(path[#path])
		circfill(g.x, g.y, 2, 8)
	end
end

function deb_rect(x1, y1, x2, y2)
	local rect_entry = {}
	rect_entry.x1 = x1
	rect_entry.y1 = y1
	rect_entry.x2 = x2
	rect_entry.y2 = y2
	add(deb_rect_buffer, rect_entry)
end

function deb_clear_buffers()
	deb_text_buffer = {}
	deb_rect_buffer = {}
	deb_log_buffer  = {}
end
-->8

-------------------------------------
---------- physics library ----------
-------------------------------------

gravity  = v_init(0, 0.4)
grd_fric = 0.5
wat_fric = 0.2

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

function phy_apply_force(obj, f)
	obj.acc.x += f.x/obj.mass
	obj.acc.y += f.y/obj.mass
end

function phy_apply_friction(obj)
	if(abs(obj.vel.x) > 0) then
		local n_vel_x = v_norm(obj.vel).x * -1
		local fric = v_init(n_vel_x * grd_fric, 0)
		phy_apply_force(obj, fric)
	end
end

function phy_apply_resitance(obj, drag_c)
	local vel_mag = v_mag(obj.vel)

	if(vel_mag > 0) then
		local n_vel   = v_mul(v_norm(obj.vel), -1)
		local resist  = v_mul(n_vel, drag_c)
		phy_apply_force(obj, v_mul(resist, vel_mag * vel_mag))
	end
end

function phy_update(obj)

	-- apply gravity
	if(obj.apply_gravity) then
		local f_gravity = v_mul(gravity, obj.mass)
		phy_apply_force(obj, f_gravity)
	end

	-- apply ground friction
	if(obj.is_on_ground) then
		phy_apply_friction(obj)
	end

	-- apply water resistance
	if(phy_is_in_water(obj.pos)) then
		phy_apply_resitance(obj, wat_fric)
	end

	-- aplly wind force
	if(phy_is_in_wind(obj.pos)) then
		local f = v_init(0,-1)
		phy_apply_force(obj, f)
	end

	--deb_text(""..obj.vel.x.." ,"..obj.vel.y, obj.pos.x, obj.pos.y - 10)

	obj.vel = v_add(obj.vel, obj.acc)

	if(abs(obj.vel.x) > 0 and obj.detect_collision) then
		phy_check_horz_coll(obj)
	end

	if(abs(obj.vel.y) > 0 and obj.detect_collision) then
		phy_check_vert_coll(obj)
	end

	-- if(abs(obj.vel.x) < 0.1) obj.vel.x = 0
	-- if(abs(obj.vel.y) < 0.1) obj.vel.y = 0

	obj.pos = v_add(obj.pos, obj.vel)
	obj.acc = v_init(0, 0)
end

function phy_is_solid(pos)
	local m_sprite = mget(pos.x, pos.y)
	return fget(m_sprite, 0)
end

function phy_is_in_water(pos)
	local center_pos = v_init(pos.x + 4, pos.y + 4)
	local map_pos = phy_screen_to_map(center_pos)
	local m_sprite = mget(map_pos.x, map_pos.y)
	return fget(m_sprite, 1)
end

function phy_is_in_wind(pos)
	local center_pos = v_init(pos.x + 4, pos.y + 4)
	local map_pos = phy_screen_to_map(center_pos)
	local m_sprite = mget(map_pos.x, map_pos.y)
	return fget(m_sprite, 2)
end

function phy_screen_to_map(pos)
	return v_init(flr(pos.x/8), flr(pos.y/8))
end

function phy_map_to_screen(pos)
	return v_mul(pos, 8)
end

function phy_check_horz_coll(obj)

	local proj_pos = v_init(obj.pos.x + obj.vel.x, obj.pos.y)

	local side = sgn(obj.vel.x)

	local t_corner = v_init(proj_pos.x, proj_pos.y)
	if(side == 1) then
		t_corner.x += 7
	end

	local b_corner = v_init(proj_pos.x, proj_pos.y + 7)
	if(side == 1) then
		b_corner.x += 7
	end

	-- transform world pos of both corners to map pos
	local t_map_pos = phy_screen_to_map(t_corner)
	local b_map_pos = phy_screen_to_map(b_corner)

	-- check in the map if the value in both map pos are solid
	if(phy_is_solid(t_map_pos) or phy_is_solid(b_map_pos)) then
		obj.vel.x = 0
  		obj.pos.x = (flr(t_map_pos.x) - side) * 8

		if(obj.did_collide_horz) obj.did_collide_horz()

	end

end

function phy_check_vert_coll(obj)

	-- project our vert position
	local proj_pos = v_init(obj.pos.x, obj.pos.y + obj.vel.y)

	-- find to which vert side i am going
	local side = sgn(obj.vel.y)

	-- find both corners on that side
	local l_corner = v_init(proj_pos.x, proj_pos.y)
	if(side == 1) then
		l_corner.y += 7
	end

	local r_corner = v_init(proj_pos.x + 7, proj_pos.y)
	if(side == 1) then
		r_corner.y += 7
	end

	-- transform world pos of both corners to map pos
	local l_map_pos = phy_screen_to_map(l_corner,8)
	local r_map_pos = phy_screen_to_map(r_corner,8)

	-- check in the map if the value in both map pos are solid
	if(phy_is_solid(l_map_pos) or phy_is_solid(r_map_pos)) then

	    obj.vel.y = 0
		obj.pos.y = (flr(l_map_pos.y) - side) * 8

		if(obj.did_collide_vert) obj.did_collide_vert()
	end

end

-------------------------------------
------ particle system library ------
-------------------------------------

function ps_part_init(sprite, mass, life_time, system)
	local part    = obj_init(system.emitter, sprite, mass)
	part.life_t   = life_time
	part.system   = system
	part.timer    = 0
	part.is_alive = false
	part.detect_collision = false
	part.apply_gravity = false
	return part
end

function ps_part_update(part)
	if(part.is_alive) then
		phy_update(part)

		part.timer += 1
		if(part.timer > part.life_t) then
			part.is_alive = false
			part.timer = 0
			part.pos = part.system.emitter
			part.vel = v_init(0,0)
			part.acc = v_init(0,0)
		end
	end
end

function ps_part_draw(part)
	if(part.is_alive) then
		spr(part.s, part.pos.x, part.pos.y)
	end
end

function ps_part_shoot(part, f)
	part.is_alive = true
	phy_apply_force(part, f)
end

function ps_system_init(sprite, pos, part_amount, shoot_freq)
	local system = {}
	system.emitter = pos
	system.particles = {}
	system.timer = 0
	system.shoot_freq = shoot_freq

	for i=1,part_amount do
		local part = ps_part_init(sprite, 1, 10, system)
		add(system.particles, part)
	end

	return system
end

function ps_system_update(system)

	system.timer += 1
	if(system.timer > system.shoot_freq) then

		local part = nil

		-- let's pick the first particle alive in the system
		for i=1,#system.particles do
			if(not system.particles[i].is_alive) then
				part = system.particles[i]
				break
			end
		end

		if(part != nil) then
			local f = v_init(-1 + rnd(2), -rnd(10))
			ps_part_shoot(part, f)
		end

		system.timer = 0
	end

	for i=1,#system.particles do
		ps_part_update(system.particles[i])
	end
end

function ps_system_draw(system)
	for i=1,#system.particles do
		ps_part_draw(system.particles[i])
	end
end
-->8

-------------------------------------
-------- animation library ----------
-------------------------------------

function anim_play(obj, anim_name, is_loop)
	if(obj.anims[anim_name] == nil) return
	if(obj.anim_loop_counter > 0 and not is_loop) return

	obj.anim_current = anim_name

	obj.anim_timer += 1
	if(obj.anim_timer > obj.anims[anim_name].anim_fr) then
		obj.s = obj.anims[anim_name].frames[obj.anim_idx]
		obj.anim_idx += 1

		if(obj.anim_idx > #obj.anims[anim_name].frames) then
			obj.anim_idx = 1
			obj.anim_loop_counter += 1
		end

		obj.anim_timer = 0
	end
end

function anim_stop(obj)
	if(obj.anims[obj.anim_current]) then
		obj.anim_timer = obj.anims[obj.anim_current].anim_fr
		obj.anim_idx = 1
		obj.anim_current = nil
	end
end

function anim_add(obj, name, frames, frame_rate)
	if(obj.anims == nil) then
		obj.anims = {}
		obj.is_animating = false
		obj.anim_idx   = 1
		obj.anim_timer = frame_rate
		obj.anim_loop_counter = 0
		obj.anim_current = name
	end

	obj.anims[name] = {}
	obj.anims[name].frames  = frames
	obj.anims[name].anim_fr = frame_rate
end
-->8

---------------------------------------------
------- steering behaviors library ----------
---------------------------------------------

function sb_seek(obj, target, r)
	local des_force = v_norm(v_sub(target, obj.pos))
	if(des_force == nil) return

	des_force = v_mul(des_force, obj.max_speed)

	local dist = v_dist(obj.pos, target)
	if(dist < r) then
		des_force = v_mul(des_force, dist/r)
	else
		des_force = v_mul(des_force, obj.max_speed)
	end

	-- calculate and apply steering force
	local sf = v_sub(des_force, obj.vel)

	-- clamp steering force
	sf = v_clamp(sf, obj.max_steering)
	phy_apply_force(obj, sf)
end

function sb_flee(obj, target)

	local des_force = v_norm(v_sub(obj.pos, target))
	if(des_force == nil) return
	des_force = v_mul(des_force, obj.max_speed)

	-- calculate and apply steering force
	local sf = v_sub(des_force, obj.vel)

	-- clamp steering force
	sf = v_clamp(sf, obj.max_steering)
	phy_apply_force(obj, sf)

end

function sb_follow_path(obj, path, r)

	if(path == nil or #path <= 1) return

	local c_min_dist = m_infinity
	local c_min_n = nil

	-- project obj position f_pos
	local f_pos = v_add(obj.pos, obj.vel)

	-- find the set of normal points
	for i=1,#path-1 do
		local s = phy_map_to_screen(path[i])
		local e = phy_map_to_screen(path[i+1])
		local n = m_project_point(s, e, f_pos)

		-- if normal point n is outside the segment (s,e)
		if(n.x < min(s.x, e.x) or n.x > max(s.x, e.x)) then
			n = e
		end

		-- pick the nearest one n
		local dist = v_dist(f_pos, n)

		if(dist < c_min_dist) then
			c_min_dist = dist
			c_min_n = n
		end
	end

	-- calc the dist d between f_pos and c_min_n
	dist = v_dist(f_pos, c_min_n)

	-- if d is greater than r, seek(c_min_n)
	if(dist > r) then
		sb_seek(obj, c_min_n, 0)
	end

end
-->8

-------------------------------------
-------pathfinding library ----------
-------------------------------------

function pf_adj(x,y)

	local adj_nodes = {}

	-- up
	local up = v_init(x, y - 1)
	if(not phy_is_solid(up)) then
		add(adj_nodes, up)
	end

	-- down
	local down = v_init(x, y + 1)
	if(not phy_is_solid(down)) then
		add(adj_nodes, down)
	end

	-- left
	local left = v_init(x - 1, y)
	if(not phy_is_solid(left)) then
		add(adj_nodes, left)
	end

	-- right
	local right = v_init(x + 1, y)
	if(not phy_is_solid(right)) then
		add(adj_nodes, right)
	end

	return adj_nodes
end

function pf_pop(queue)
	if(#queue > 0) then
		local f_elem = queue[1]
		del(queue, f_elem)
		return f_elem
	end

	return nil
end

function pf_d_pop(queue, dist)
	local pop_elem = nil

	if(#queue > 0) then
		local c_node = queue[1]
		local c_min = dist[v_str(c_node)]
		pop_elem = c_node

		for i=1,#queue do
			c_node = queue[i]
			if(dist[v_str(c_node)] < c_min) then
				c_min = dist[v_str(c_node)]
				pop_elem = c_node
			end
		end

		del(queue, pop_elem)
	end

	return pop_elem
end

function pf_path(s, g, back_p)
	local c_node = g
	local path = {}

	while(v_str(c_node) != v_str(s)) do
		add(path, c_node)
		c_node = back_p[v_str(c_node)]
	end

	-- reverse path
	for i=1,#path/2 do
		aux = path[i]
		path[i]=path[#path - i + 1]
		path[#path - i + 1] = aux
	end

	return path
end

function pf_cost(a, b, cost_flags)
	local cost_a = 1
	local cost_b = 1
	for flag in all(cost_flags) do
		local a_spr = mget(a.x, a.y)
		if(fget(a_spr, flag)) cost_a += flag

		local b_spr = mget(b.x, b.y)
		if(fget(b_spr, flag)) cost_b += flag
	end
	return cost_a + cost_b
end

function pf_dfs(s,g)
	local queue = {}
	add(queue, s)

	local back_p = {}
	back_p[v_str(s)] = s

	local dist = {}
	dist[v_str(s)] = 0

	while(#queue > 0) do
		local c_node = pf_d_pop(queue, dist)

		-- if i found the goal, return the path
		if(v_str(c_node) == v_str(g)) then
			return pf_path(s,g,back_p)
		end

		-- get the list of adj nodes
		local adj_nodes = pf_adj(c_node.x, c_node.y)

		-- iterate on the list of adj nodes
		for i=1,#adj_nodes do
			local c_adj = adj_nodes[i]
			local path_cost = dist[v_str(c_node)] + pf_cost(c_node, c_adj, {4,5,6,7})

			if(dist[v_str(c_adj)] == nil or path_cost < dist[v_str(c_adj)]) then
				add(queue, c_adj)
				dist[v_str(c_adj)] = path_cost
				back_p[v_str(c_adj)] = c_node
			end
		end
	end
end

function pf_a_star(s,g)
	local queue = {}
	add(queue, s)

	local back_p = {}
	back_p[v_str(s)] = s

	local dist = {}
	dist[v_str(s)] = 0

	while(#queue > 0) do
		local c_node = pf_d_pop(queue, dist)

		-- if i found the goal, return the path
		if(v_str(c_node) == v_str(g)) then
			return pf_path(s,g,back_p)
		end

		-- get the list of adj nodes
		local adj_nodes = pf_adj(c_node.x, c_node.y)

		-- iterate on the list of adj nodes
		for i=1,#adj_nodes do
			local c_adj = adj_nodes[i]
			local path_cost = dist[v_str(c_node)] + pf_cost(c_node, c_adj, {4,5,6,7})

			if(dist[v_str(c_adj)] == nil or path_cost + v_dist(c_adj, g) < dist[v_str(c_adj)]) then
				add(queue, c_adj)
				dist[v_str(c_adj)] = path_cost + v_dist(c_adj, g)
				back_p[v_str(c_adj)] = c_node
			end
		end
	end
end


function pf_bfs(s,g)

	local queue = {}
	add(queue, s)

	local back_p = {}
	back_p[v_str(s)] = s

	while(#queue > 0) do
		local c_node = pf_pop(queue)

		-- if i found the goal, return the path
		if(v_str(c_node) == v_str(g)) then
			return pf_path(s,g,back_p)
		end

		-- get the list of adj nodes
		local adj_nodes = pf_adj(c_node.x, c_node.y)

		-- iterate on the list of adj nodes
		for i=1,#adj_nodes do
			local c_adj = adj_nodes[i]

			-- check if i've visited this node before
			if(back_p[v_str(c_adj)] == nil) then
				add(queue, c_adj)
				back_p[v_str(c_adj)] = c_node
			end
		end
	end

end
-->8

-------------------------------------
------------ fsm library ------------
-------------------------------------

function fsm_init(states, s_state, parent)
	local fsm = nil
	if(states[s_state] != nil) then
		fsm = {}
		fsm.c_state = s_state
		fsm.states = states
		fsm.parent = parent
	end
	return fsm
end

function fsm_state_init(name, f_update, f_enter, f_exit)
	local state = {}
	state.name   = name
	state.update = f_update
	state.enter  = f_enter
	state.exit   = f_exit
	state.timer  = 0
	return state
end

function fsm_update(fsm)
	if(fsm.states[fsm.c_state] != nil) then
		fsm.states[fsm.c_state].update(fsm.parent)

	end
end

function fsm_change_state(fsm, new_state)
	if(fsm.states[fsm.c_state].exit != nil) then
		fsm.states[fsm.c_state].exit(fsm.parent)
	end

	if(fsm.states[new_state] != nil) then
		fsm.c_state = new_state
		fsm.states[fsm.c_state].timer = 0

		if(fsm.states[fsm.c_state].enter != nil) then
			fsm.states[fsm.c_state].enter(fsm.parent)
		end
	end
end
-->8

-------------------------------------
-------------- main game ------------
-------------------------------------

function obj_init(pos, s, mass, collider)
	local game_obj = {}
	game_obj.s    = s
	game_obj.pos  = pos
	game_obj.vel  = v_init(0,0)
	game_obj.acc  = v_init(0,0)
	game_obj.mass = mass
	game_obj.max_speed = 1
	game_obj.max_steering = 1
	game_obj.box  = collider
	game_obj.is_flipped = false
	game_obj.detect_collision = true
	game_obj.apply_gravity = true
	game_obj.is_on_ground = true
	game_obj.did_collide_horz = nil
	game_obj.did_collide_vert = nil
	return game_obj
end

map_width = 128
map_height = 64

function _init()

	enemies = {}

	-- player position
	for x=0,map_width do
		for y=0,map_height do
			local m_tile = mget(x,y)

			local pos = phy_map_to_screen(v_init(x,y))

			-- sprite 1 is the player
			if(m_tile == 1) then

				local col = phy_aabb_init(2,2,3,6)
				player = obj_init(pos, m_tile, 1, col)
				player.apply_gravity = false
				player.is_on_ground = false
				anim_add(player, "move", {17,33}, 8)
				mset(x,y,16)

			-- sprite 32 is the enemy
			elseif(m_tile == 32) then
				local col = phy_aabb_init(1,0,6,8)
				local enemy = obj_init(pos, m_tile, 1, col)
				enemy.apply_gravity = false
				enemy.is_on_ground = false
				enemy.max_speed = 0.85
				enemy.max_steering = 0.1
				anim_add(enemy, "move", {32, 48}, 10)

				-- creating fsm
				local states = {}
				states["chase"] = fsm_state_init("chase", enemy_chase_update, nil, nil)
				states["flee"]  = fsm_state_init("flee", enemy_flee_update, nil, nil)
				enemy.fsm = fsm_init(states, "chase", enemy)

				add(enemies, enemy)
				mset(x,y,16)
			end
		end
	end

	wind = ps_system_init(20, v_init(320,160), 32, 5)

	camera_pos = v_init(0,0)

	-- create a menu item
	menuitem(1,"debug mode",deb_start)
end

function _update()

	if(deb_mode_on) then

		-- enable the cursor
		poke(0x5f2d, 1)

		-- if in debug mode, change debug state to slowmo if player presses a
		if(btnp(5,1)) then
			deb_curt_state += 1
			if(deb_curt_state > #deb_states) then
				deb_curt_state = 1
			end
		end

		if(deb_states[deb_curt_state] == "normal") then
			-- if in debug mode normal, call update normally
			__update()
		elseif(deb_states[deb_curt_state] == "step") then
			-- if in debug mode step, call update whenever player presses key a
			if(btnp(0,1)) then
				__update()
			end
		elseif(deb_states[deb_curt_state] == "slowmo") then
			-- if in debug mode slowmo, call update after every 10th frame
			deb_frame_timer += 1
			if(deb_frame_timer > 5) then
				__update()
				deb_frame_timer = 0
			end
		end

	else
		-- disable the cursor
		poke(0x5f2d, 0)

		-- if not in debug mode, just update game normally
		__update()
	end
end

function __update()

	player_update()
	foreach(enemies, enemy_update)
end

function enemy_update(enemy)
	fsm_update(enemy.fsm)
	phy_update(enemy)
end

function enemy_flee_update(enemy)

	local c_state = enemy.fsm.c_state
	fade((10 - enemy.fsm.states[c_state].timer)/10)

	enemy.fsm.states[c_state].timer += 1
	if(enemy.fsm.states[c_state].timer > 90) then
		fsm_change_state(enemy.fsm, "chase")
		enemy.fsm.states[c_state].timer = 0
	end

	sb_flee(enemy, player.pos)
end

function enemy_chase_update(enemy)

	if(phy_aabb_collide(enemy.pos, enemy.box, player.pos, player.box)) then

		local c_state = enemy.fsm.c_state
		fade(enemy.fsm.states[c_state].timer/10)

		enemy.fsm.states[c_state].timer += 1
		if(enemy.fsm.states[c_state].timer > 90) then
			fsm_change_state(enemy.fsm, "flee")
			fade(1)
			enemy.fsm.states[c_state].timer = 0
		end
	end

	local sb_radius = 5

	path = pf_bfs(phy_screen_to_map(enemy.pos),
			 	  phy_screen_to_map(player.pos))

	if(path != nil and #path > 1) then
		sb_follow_path(enemy, path, sb_radius)
	else
		sb_seek(enemy, player.pos, sb_radius)
	end

	anim_play(enemy, "move", true)
end

-- player functions
function player_update()

	player.vel.x = 0
	player.vel.y = 0

	if(btn(0)) player.vel.x += -1
	if(btn(1)) player.vel.x +=  1
	if(btn(2)) player.vel.y += -1
	if(btn(3)) player.vel.y +=  1

	if(btn(0) or btn(1) or btn(2) or btn(3)) then
		anim_play(player, "move", true)
	else
		player.s = 1
	end

	phy_update(player)
end

function _draw()

	__draw()

	local cursor_pos = v_init(stat(32)-1, stat(33)-1)

	-- draw debug tools
	if(deb_mode_on) then

		local cursor_pos = v_init(stat(32)-1, stat(33)-1)

		-- draw the console
		local console_y = 100

		if(cursor_pos.y > console_y) then
			rectfill(camera_pos.x + 0  , camera_pos.y + 100,
					 camera_pos.x + 128, camera_pos.y + 128, 2)

			print("console", camera_pos.x + 1, camera_pos.y + console_y + 2, 7)

			-- draw log messages
			local log_y = console_y + 10
			for i=1,#deb_log_buffer do
				print(deb_log_buffer[i], camera_pos.x + 1, camera_pos.y + log_y, 7)
				log_y += 10
			end
		end

		-- draw the texts
		for i=1,#deb_text_buffer do
			print(deb_text_buffer[i].msg, deb_text_buffer[i].x,
										  deb_text_buffer[i].y, 7)
		end

		-- draw the rects
		for i=1,#deb_rect_buffer do
			rect(deb_rect_buffer[i].x1, deb_rect_buffer[i].y1,
				 deb_rect_buffer[i].x2, deb_rect_buffer[i].y2, 7)
		end

		print("#debug on: "..deb_states[deb_curt_state], camera_pos.x + 1, camera_pos.y + 1, 2)
		if(deb_states[deb_curt_state] == "step") then
			print("#press s to update", camera_pos.x + 1, camera_pos.y + 10, 2)
		elseif(deb_states[deb_curt_state] == "slowmo") then
			print("#debug on: slowmo", camera_pos.x + 1, camera_pos.y + 1, 2)
		end

		-- draw the cursor
		spr(0, camera_pos.x + cursor_pos.x, camera_pos.y + cursor_pos.y)

		-- draw cpu and memory stats
		print("mem: ".. stat(0), camera_pos.x + 80, camera_pos.y + 1)
		print("cpu: ".. stat(1), camera_pos.x + 80, camera_pos.y + 10)

		deb_path(path)

		deb_clear_buffers()
	end

end

function obj_draw(obj)
	spr(obj.s, obj.pos.x, obj.pos.y, 1, 1, obj.is_flipped, false)
end

function __draw()

	cls()

	map(0,0,0,0,128,64)

	ps_system_draw(wind)
	foreach(enemies, obj_draw)

	-- drawing the player
	pal(7, 7, 0)
	obj_draw(player)
end

__gfx__
00000000008888004444444499999944449999999999999944444444444444440000000000000000000000000000000000000000000000000000000000000000
00000000088888004444444499999944449999999999999944444444444444440000000000000000000000000000000000000000000000000000000000000000
0070070088fff8809999999999999944449999999999999944999999999999440000000000000000000000000000000000000000000000000000000000000000
00077000887f78809999999999999944449999999999999944999999999999440000000000000000000000000000000000000000000000000000000000000000
0007700008fff8009999999999999944449999999999999944999999999999440000000000000000000000000000000000000000000000000000000000000000
00700700009990009999999999999944449999999999999944999999999999440000000000000000000000000000000000000000000000000000000000000000
00000000004440009999999999999944449999994444444444999999999999440000000000000000000000000000000000000000000000000000000000000000
0000000000f0f0009999999999999944449999994444444444999999999999440000000000000000000000000000000000000000000000000000000000000000
000000000088880044444444cccccccc000000001111111144999999999999440000000000000000000000000000000000000000000000000000000000000000
000000000888880049444444cccccccc000000001111111144999999999999440000000000000000000000000000000000000000000000000000000000000000
0000000088fff88044444494cccccccc000000001111111144999999999999440000000000000000000000000000000000000000000000000000000000000000
00000000887f788044494444cccccccc000770001111111144999999999999440000000000000000000000000000000000000000000000000000000000000000
0000000008fff80044444444cccccccc000770001111111144999999999999440000000000000000000000000000000000000000000000000000000000000000
000000000099900044444444cccccccc000000001111111144999999999999440000000000000000000000000000000000000000000000000000000000000000
000000000044400049444944cccccccc000000001111111144444444444444440000000000000000000000000000000000000000000000000000000000000000
0000000000f0000044444444cccccccc000000001111111144444444444444440000000000000000000000000000000000000000000000000000000000000000
00022000008888009999999988888888000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
00222200088888009999999988888888000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
0227272088fff8809999999988888888000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
02272720887f78809999999988888888000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
0222222008fff8009999999988888888000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
00222220009990009999999988888888000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
00222200004440009999999988888888000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
000220000000f0009999999988888888000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
00022000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
00222200000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
02262620000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
02262620000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
02222220000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
00222220000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
00222200000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
__label__
44444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444
49444444494444444944444449444444494444444944444449444444494444444944444449444444494444444944444449444444494444444944444449444444
44444494444444944444449444444494444444944444449444444494444444944444449444444494444444944444449444444494444444944444449444444494
44494444444944444449444444494444444944444449444444494444444944444449444444494444444944444449444444494444444944444449444444494444
44444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444
44444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444
49444944494449444944494449444944494449444944494449444944494449444944494449444944494449444944494449444944494449444944494449444944
44444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444
44444444000000000000000000000000000000000000000000000000000000000000000044444444000000000000000000000000444444444444444444444444
49444444000000000000000000000000000000000000000000000000000000000000000049444444000000000000000000000000494444444944444449444444
44444494000000000000000000000000000000000000000000000000000000000000000044444494000000000000000000000000444444944444449444444494
44494444000000000000000000000000000000000000000000000000000000000000000044494444000000000000000000000000444944444449444444494444
44444444000000000000000000000000000000000000000000000000000000000000000044444444000000000000000000000000444444444444444444444444
44444444000000000000000000000000000000000000000000000000000000000000000044444444000000000000000000000000444444444444444444444444
49444944000000000000000000000000000000000000000000000000000000000000000049444944000000000000000000000000494449444944494449444944
44444444000000000000000000000000000000000000000000000000000000000000000044444444000000000000000000000000444444444444444444444444
44444444000000000000000000000000000000000000000000000000000000000000000044444444000000000000000000000000444444444444444444444444
49444444000000000000000000000000000000000000000000000000000000000000000049444444000000000000000000000000494444444944444449444444
44444494000000000000000000000000000000000000000000000000000000000000000044444494000000000000000000000000444444944444449444444494
44494444000000000000000000000000000000000000000000000000000000000000000044494444000000000000000000000000444944444449444444494444
44444444000000000000000000000000000000000000000000000000000000000000000044444444000000000000000000000000444444444444444444444444
44444444000000000000000000000000000000000000000000000000000000000000000044444444000000000000000000000000444444444444444444444444
49444944000000000000000000000000000000000000000000000000000000000000000049444944000000000000000000000000494449444944494449444944
44444444000000000000000000000000000000000000000000000000000000000000000044444444000000000000000000000000444444444444444444444444
44444444000000000000000044444444000000000000000000000000000000000000000000000000000000000000000000000000000000000000000044444444
49444444000000000000000049444444000000000000000000000000000000000000000000000000000000000000000000000000000000000000000049444444
44444494000000000000000044444494000000000000000000000000000000000000000000000000000000000000000000000000000000000000000044444494
44494444000000000000000044494444000000000000000000000000000000000000000000000000000000000000000000000000000000000000000044494444
44444444000000000000000044444444000000000000000000000000000000000000000000000000000000000000000000000000000000000000000044444444
44444444000000000000000044444444000000000000000000000000000000000000000000000000000000000000000000000000000000000000000044444444
49444944000000000000000049444944000000000000000000000000000000000000000000000000000000000000000000000000000000000000000049444944
44444444000000000000000044444444000000000000000000000000000000000000000000000000000000000000000000000000000000000000000044444444
44444444000000000000000044444444444444444444444444444444000000000000000000000000000000000002200000000000000000000000000044444444
49444444000000000000000049444444494444444944444449444444000000000000000000000000000000000022220000000000000000000000000049444444
44444494000000000000000044444494444444944444449444444494000000000000000000000000000000000227272000000000000000000000000044444494
44494444000000000000000044494444444944444449444444494444000000000000000000000000000000000227272000000000000000000000000044494444
44444444000000000000000044444444444444444444444444444444000000000000000000000000000000000222222000000000000000000000000044444444
44444444000000000000000044444444444444444444444444444444000000000000000000000000000000000022222000000000000000000000000044444444
49444944000000000000000049444944494449444944494449444944000000000000000000000000000000000022220000000000000000000000000049444944
44444444000000000000000044444444444444444444444444444444000000000000000000000000000000000002200000000000000000000000000044444444
44444444000000000000000044444444444444444444444444444444000000000000000000000000000000000000000000000000000000004444444444444444
49444444000000000000000049444444494444444944444449444444000000000000000000000000000000000000000000000000000000004944444449444444
44444494000000000000000044444494444444944444449444444494000000000000000000000000000000000000000000000000000000004444449444444494
44494444000000000000000044494444444944444449444444494444000000000000000000000000000000000000000000000000000000004449444444494444
44444444000000000000000044444444444444444444444444444444000000000000000000000000000000000000000000000000000000004444444444444444
44444444000000000000000044444444444444444444444444444444000000000000000000000000000000000000000000000000000000004444444444444444
49444944000000000000000049444944494449444944494449444944000000000000000000000000000000000000000000000000000000004944494449444944
44444444000000000000000044444444444444444444444444444444000000000000000000000000000000000000000000000000000000004444444444444444
44444444000000000000000044444444000000000000000000000000000000000000000000000000000000004444444400000000000000000000000044444444
49444444000000000000000049444444000000000000000000000000000000000000000000000000000000004944444400000000000000000000000049444444
44444494000000000000000044444494000000000000000000000000000000000000000000000000000000004444449400000000000000000000000044444494
44494444000000000000000044494444000000000000000000000000000000000000000000000000000000004449444400000000000000000000000044494444
44444444000000000000000044444444000000000000000000000000000000000000000000000000000000004444444400000000000000000000000044444444
44444444000000000000000044444444000000000000000000000000000000000000000000000000000000004444444400000000000000000000000044444444
49444944000000000000000049444944000000000000000000000000000000000000000000000000000000004944494400000000000000000000000049444944
44444444000000000000000044444444000000000000000000000000000000000000000000000000000000004444444400000000000000000000000044444444
44444444000000000000000000000000000000000000000000000000000000000000000000000000000000004444444444444444000000000000000044444444
49444444000000000000000000000000000000000000000000000000000000000000000000000000000000004944444449444444000000000000000049444444
44444494000000000000000000000000000000000000000000000000000000000000000000000000000000004444449444444494000000000000000044444494
44494444000000000000000000000000000000000000000000000000000000000000000000000000000000004449444444494444000000000000000044494444
44444444000000000000000000000000000000000000000000000000000000000000000000000000000000004444444444444444000000000000000044444444
44444444000000000000000000000000000000000000000000000000000000000000000000000000000000004444444444444444000000000000000044444444
49444944000000000000000000000000000000000000000000000000000000000000000000000000000000004944494449444944000000000000000049444944
44444444000000000000000000000000000000000000000000000000000000000000000000000000000000004444444444444444000000000000000044444444
44444444000000000000000000000000000000000000000000000000000000000000000000000000000000000000000044444444000000000000000044444444
49444444000000000000000000000000000000000000000000000000000000000000000000000000000000000000000049444444000000000000000049444444
44444494000000000000000000000000000000000000000000000000000000000000000000000000000000000000000044444494000000000000000044444494
44494444000000000000000000000000000000000000000000000000000000000000000000000000000000000000000044494444000000000000000044494444
44444444000000000000000000000000000000000000000000000000000000000000000000000000000000000000000044444444000000000000000044444444
44444444000000000000000000000000000000000000000000000000000000000000000000000000000000000000000044444444000000000000000044444444
49444944000000000000000000000000000000000000000000000000000000000000000000000000000000000000000049444944000000000000000049444944
44444444000000000000000000000000000000000000000000000000000000000000000000000000000000000000000044444444000000000000000044444444
44444444000000000000000044444444444444444444444444444444444444444444444400000000000000000000000044444444000000000000000044444444
49444444000000000000000049444444494444444944444449444444494444444944444400000000000000000000000049444444000000000000000049444444
44444494000000000000000044444494444444944444449444444494444444944444449400000000000000000000000044444494000000000000000044444494
44494444000000000000000044494444444944444449444444494444444944444449444400000000000000000000000044494444000000000000000044494444
44444444000000000000000044444444444444444444444444444444444444444444444400000000000000000000000044444444000000000000000044444444
44444444000000000000000044444444444444444444444444444444444444444444444400000000000000000000000044444444000000000000000044444444
49444944000000000000000049444944494449444944494449444944494449444944494400000000000000000000000049444944000000000000000049444944
44444444000000000000000044444444444444444444444444444444444444444444444400000000000000000000000044444444000000000000000044444444
44444444000000000000000044444444444444444444444444444444444444444444444400000000000000004444444444444444000000000000000044444444
49444444000000000000000049444444494444444944444449444444494444444944444400000000000000004944444449444444000000000000000049444444
44444494000000000000000044444494444444944444449444444494444444944444449400000000000000004444449444444494000000000000000044444494
44494444000000000000000044494444444944444449444444494444444944444449444400000000000000004449444444494444000000000000000044494444
44444444000000000000000044444444444444444444444444444444444444444444444400000000000000004444444444444444000000000000000044444444
44444444000000000000000044444444444444444444444444444444444444444444444400000000000000004444444444444444000000000000000044444444
49444944000000000000000049444944494449444944494449444944494449444944494400000000000000004944494449444944000000000000000049444944
44444444000000000000000044444444444444444444444444444444444444444444444400000000000000004444444444444444000000000000000044444444
44444444000000000000000000000000000000004444444444444444444444440000000000000000000000004444444400000000000000000000000044444444
49444444000000000000000000000000000000004944444449444444494444440000000000000000000000004944444400000000000000000000000049444444
44444494000000000000000000000000000000004444449444444494444444940000000000000000000000004444449400000000000000000000000044444494
44494444000000000000000000000000000000004449444444494444444944440000000000000000000000004449444400000000000000000000000044494444
44444444000000000000000000000000000000004444444444444444444444440000000000000000000000004444444400000000000000000000000044444444
44444444000000000000000000000000000000004444444444444444444444440000000000000000000000004444444400000000000000000000000044444444
49444944000000000000000000000000000000004944494449444944494449440000000000000000000000004944494400000000000000000000000049444944
44444444000000000000000000000000000000004444444444444444444444440000000000000000000000004444444400000000000000000000000044444444
44444444000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000044444444
49444444000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000049444444
44444494000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000044444494
44494444000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000044494444
44444444000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000044444444
44444444000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000044444444
49444944000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000049444944
44444444000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000044444444
44444444000000000000000000888800000000000000000000000000000000000000000000000000000000000000000000000000000000000000000044444444
49444444000000000000000008888800000000000000000000000000000000000000000000000000000000000000000000000000000000000000000049444444
44444494000000000000000088fff880000000000000000000000000000000000000000000000000000000000000000000000000000000000000000044444494
444944440000000000000000881f1880000000000000000000000000000000000000000000000000000000000000000000000000000000000000000044494444
44444444000000000000000008fff800000000000000000000000000000000000000000000000000000000000000000000000000000000000000000044444444
44444444000000000000000000999000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000044444444
49444944000000000000000000444000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000049444944
44444444000000000000000000f0f000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000044444444
44444444000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000044444444
49444444000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000049444444
44444494000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000044444494
44494444000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000044494444
44444444000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000044444444
44444444000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000044444444
49444944000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000049444944
44444444000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000044444444
44444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444
49444444494444444944444449444444494444444944444449444444494444444944444449444444494444444944444449444444494444444944444449444444
44444494444444944444449444444494444444944444449444444494444444944444449444444494444444944444449444444494444444944444449444444494
44494444444944444449444444494444444944444449444444494444444944444449444444494444444944444449444444494444444944444449444444494444
44444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444
44444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444
49444944494449444944494449444944494449444944494449444944494449444944494449444944494449444944494449444944494449444944494449444944
44444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444

__gff__
0000010101010101000000000000000010000120004001010000000000000000800000800000000000000000000000008000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
0000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
__map__
1212121212121212121212121212121210101010101010101010101010000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
1210101010101010101210101012121210101010101010101010101010000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
1210101010101010101210101012121210101010101010101010101010001010101000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
1210101210101010101010101010101210101010101010101010101010101010101010101000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
1210101212121210101010101010101210101010101010101010101010101010101010101000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
1210101212121210101010101010121210101010101010101010101010101010101010101000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
1210101210101010101010121010101210101000001010101000000010100000001010101000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
1210101010101010101010121210101210100000000010101000000000000000000010100000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
1210101010101010101010101210101210000000000000100000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
1210101212121212121010101210101210000000000000000000000000000000000000000000101010000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
1210101212121212121010121210101210000000000000000000000000000000000000000000000000000000000010000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
1210101010121212101010121010101210000000000000000000000000000000000000000000000000000010101010101010101010000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
1210101010101010101010101010101210000000000000100000000000100000000000000000101010101010101010101010101010000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
1210100110201010101010101010101210000000000000101010101010100000000000000000101010101010101010101010101010000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
1210101010101010101010101010101210101000000010101010101010101000000000000000101010101010101010101010101010000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
1212121212121212121212121212121210101010101010101010101010101010101010101010101010101010101010101010101010000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
1010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010101010000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
1010101010101010101010101010101010101010101010101010101010101010101010101010000000101010101010101010101010000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
1010101010101010101010101010101010101010101010101010101010101010101010101010000000101010101010101010101010000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
1010101010101010101010101010101010101010101010101010101010101010101010101010000000000010101010101010101010000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
1010101010101010101010101010101010101010101010101010101010101010101010101010000000000010101010101010101010000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
1010101010101010101010101010101010101010101010101010101010101010101010101010000000000010101010101010101010000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
1010101010101010101010101010101010101010101010101010101010101010101010101010000000000010101010101010101010000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
1010101010101010101010101010101010101010101010101010101010101010101010101010000000000010101010101010101010000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
1010101010101010101010101010101010101010101010101010101010101010101010101010000000000010101010101010101010000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
0000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
0000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
0000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
0000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
0000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
0000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
0000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
__sfx__
012500001c3511d3011d301183011c2001c3001c4001c5001c6001c7001c6001a600186001c604000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
0112001018152181021825218102182551a1021a4561a1021a1561a1021a4561c1021c4561c1011c1011c10200000000000000000000000000000000000000000000000000000000000000000000000000000000
013e0000101431c5511c303185061d1021d1021d1020c0001c1021c1021c102000001a1021a1021a1020000018102181021810200000000000000000000000000000000000000000000000000000000000000000
001000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
001000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
001000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
001000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
001000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
001000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
001000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
001000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
001000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
001000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
001000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
001000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
001000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
001000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
001000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
001000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
001000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
001000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
001000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
001000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
001000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
001000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
001000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
001000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
001000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
001000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
001000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
001000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
001000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
001000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
001000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
001000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
001000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
001000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
001000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
001000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
001000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
001000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
001000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
001000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
001000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
001000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
001000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
001000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
001000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
001000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
001000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
001000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
001000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
001000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
001000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
001000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
001000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
001000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
001000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
001000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
001000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
001000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
001000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
001000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
001000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
__music__
00 01024344
00 41424344
00 41424344
00 41424344
00 41424344
00 41424344
00 41424344
00 41424344
00 41424344
00 41424344
00 41424344
00 41424344
00 41424344
00 41424344
00 41424344
00 41424344
00 41424344
00 41424344
00 41424344
00 41424344
00 41424344
00 41424344
00 41424344
00 41424344
00 41424344
00 41424344
00 41424344
00 41424344
00 41424344
00 41424344
00 41424344
00 41424344
00 41424344
00 41424344
00 41424344
00 41424344
00 41424344
00 41424344
00 41424344
00 41424344
00 41424344
00 41424344
00 41424344
00 41424344
00 41424344
00 41424344
00 41424344
00 41424344
00 41424344
00 41424344
00 41424344
00 41424344
00 41424344
00 41424344
00 41424344
00 41424344
00 41424344
00 41424344
00 41424344
00 41424344
00 41424344
00 41424344
00 41424344
00 41424344
