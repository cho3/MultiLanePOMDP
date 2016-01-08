#visualization.jl
#the stuff to visualize a state-action pair

#TODO: simple testing--e.g. make sure it runs properly--will do when I build
#				the simulator

using PyPlot #make my life easy

draw_box(x::Union{Float64,Int},y::Union{Float64,Int},w::Union{Float64,Int},h::Union{Float64,Int})=fill([x;x+w;x+w;x],[y;y;y+h;y+h])
draw_box(x::Union{Float64,Int},y::Union{Float64,Int},w::Union{Float64,Int},h::Union{Float64,Int},color::AbstractString)=fill([x;x+w;x+w;x],[y;y;y+h;y+h],color=color)

function draw_bang(x::Union{Float64,Int},
										y::Union{Float64,Int},
										color::AbstractString="#FFBF00";
    								a::Union{Float64,Int}=2.,
										b::Union{Float64,Int}=a,
    								nb_spikes::Int=8,
										spike_depth::Union{Float64,Int}=1.5,
										th_offset::Union{Float64,Int}=0.,
    								th_irregularity::Float64=0.5,
										r_irregularity::Float64=0.5,
										rng::AbstractRNG=MersenneTwister(1),
    								edge_color::AbstractString="k")
    #A somewhat overly complicated function for drawing the crash graphic
    X = zeros(nb_spikes*2)
    Y = zeros(nb_spikes*2)

    th_interval = 2.*pi/length(X)

    for i = 1:length(X)
        if i % 2 == 1
            a_ = a+spike_depth*(0.5 + r_irregularity*(rand(rng)-0.5))
            b_ = b+spike_depth*(0.5 + r_irregularity*(rand(rng)-0.5))
        else
            a_ = a-spike_depth*(0.5 + r_irregularity*(rand(rng)-0.5))
            b_ = b-spike_depth*(0.5 + r_irregularity*(rand(rng)-0.5))
        end
        th = (i+th_irregularity*(rand(rng)-0.5))*th_interval+th_offset
        X[i] = x + a_*cos(th)
        Y[i] = y + b_*sin(th)
    end

    fill(X,Y,color=color,ec=edge_color)
end

function draw_sedan(pomdp::MLPOMDP,x_ctr::Float64,y_ctr::Float64,color::AbstractString)
	#x_ctr = pomdp.POSITIONS[s.pos[1]]
	#y_ctr = pomdp.phys_param.y_interval*s.pos[2]
	w = pomdp.phys_param.l_car
	h = pomdp.phys_param.w_car
	x = x_ctr - 0.5*w
	y = y_ctr - 0.5*h

	##TODO: color stuff based on behavior Model
	draw_box(x,y,w,h,color)
	window_color = "#848484"
	x_windshield = x_ctr
	w_windshield = w*0.25
	h_windshield = h*0.75
	y_windshield = y_ctr - 0.5*h_windshield
	draw_box(x_windshield,y_windshield,w_windshield,h_windshield,window_color)
	x_backwindow = x_ctr - 0.375*w
	w_backwindow = w*0.125
	h_backwindow = w*0.75
	y_backwindow = y_ctr - 0.5*h_backwindow
	draw_box(x_backwindow,y_backwindow,w_backwindow,h_backwindow,window_color)
	##TODO: add direction arrow
	##TODO: add a face based on behavior model
end

function draw_direction(pomdp::MLPOMDP,x::Float64,y::Float64,v_nom::Float64,s::CarState)
	#plot desired car speed relative to self car as an opaque arrow
	#arrow should be at most one car in length, and ~1.2 of a car width at headway
	#NOTE: the arrow head is extra length
	max_arrow_length = 1.125*pomdp.phys_param.l_car
	phantom_arrow_length = max_arrow_length*(pomdp.VELOCITIES[s.vel]-v_nom)/(pomdp.phys_param.v_fast-pomdp.phys_param.v_slow)
	phantom_arrow_head_width = 1.2*pomdp.phys_param.w_car
	phantom_arrow_body_width  = 0.8*pomdp.phys_param.w_car
	phantom_arrow_head_length = phantom_arrow_length/3.
	arrow(x,y,phantom_arrow_length,0.,head_width=phantom_arrow_head_width,
				head_length=phantom_arrow_head_width,width=phantom_arrow_body_width,
				alpha=0.5,color="#1C1C1C")

	#draw actual speed/direction; this is where the center of the car will be in
	#the next time step, approximately

	dx = pomdp.phys_param.dt*(pomdp.VELOCITIES[s.vel]-v_nom)
	dy = s.lane_change*pomdp.phys_param.y_interval
	hw = 0.5*pomdp.phys_param.w_car
	hl = 1.5*hw
	w = 0.75*hw
	th = atan2(dy,dx)
	dx -= hl*cos(th)
	dy -= hl*sin(th)
	arrow(x,y,dx,dy,width=w,head_width=hw,head_length=hw,fc="#DF7401", ec="#0404B4",alpha=0.75)
	#terrible looking high contrast blue/orange arrows

end

function draw_sedan(pomdp::MLPOMDP,s::CarState,v_nom::Float64)
	x_ctr = pomdp.POSITIONS[s.pos[1]]
	y_ctr = pomdp.phys_param.y_interval*s.pos[2]
	##TODO: something to do with behavior and color
	color = "" #PLACEHOLDER
	draw_direction(pomdp,x,y,v_nom,s)
	draw_sedan(pomdp,x_ctr,y_ctr,color)
	#draw_direction?
	#draw_ face?
end

function visualize(pomdp::MLPomdp,s::MLState,a::MLAction)
	#Placeholder!
	#make the initial canvas
	W = pomdp.phys_param.lane_length
	H = pomdp.phys_param.w_lane*(pomdp.nb_col+1)/2
	draw_box(0,0,W,H,"#424242") #a gray

	#draw hard borders on top and bottom
	border_thickness = 2.
	border_color = "#000000" #black
	draw_box(0,-border_thickness,W,border_thickness,border_color)
	draw_box(0.,H,W,border_thickness,border_color)

	#draw the lane markings 3ft long, 9ft btwn (bikes), 10cm wide
	#~1m x 0.1m for an approximation
	lane_marker_length = 1.
	lane_marker_width = 0.1
	lane_marker_inbetween = 4. #1+3

	lane_marker_x = collect((0.-rand()):lane_marker_inbetween:W)
	lane_marker_length = Float64[lane_marker_length for i=1:length(lane_mark_x)]
	lane_marker_length[1] += lane_marker_x[1]
	lane_marker_length[end] = lane_marker_length[end]+lane_marker_x[end] > W? W-lane_marker_x[end]:lane_marker_length[end]
	lane_marker_x[1] = max(0.,lane_marker_x[1])
	lane_marker_y = collect(pomdp.phys_param.w_lane:pomdp.phys_param.w_lane:(H-(pomdp.phys_param.w_lane/2.)))

	for (i,x) in enumerate(lane_marker_x)
    for y in lane_marker_y
        draw_box(x,y,lane_marker_length[i],lane_marker_width,"#F2F2F2") #off-white
    end
	end

	#draw self car
	x_ctr = W/2.
	y_ctr = pomdp.phys_param.y_interval*s.pos
	color = "" #PLACEHOLDER
	draw_sedan(pomdp,x_ctr,y_ctr,color)
	####TODO: move this to a separate function, but it's here for now because I'm lazy
	hw = 0.5*pomdp.phys_param.w_car
	hl = 1.5*hw
	w = 0.75*hw
	dx = 0.
	dy = a.lane_change*pomdp.phys_param.y_interval - hl
	arrow(x,y,dx,dy,width=w,head_width=hw,head_length=hw,fc="#DF7401", ec="#0404B4",alpha=0.75)
	####END TODO

	v_nom = pomdp.VELOCITIES[s.vel]
	#draw environment cars
	for car in s.env_cars
		draw_sedan(pomdp,car,v_nom)
	end

	#if is_crash, add crash graphic
	if is_crash(pomdp,s,a)
		draw_bang(x_ctr,y_ctr)
	end

	axis("equal")
end
