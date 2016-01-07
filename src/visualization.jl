#visualization.jl
#the stuff to visualize a state-action pair

using PyPlot #make my life easy

draw_box(x::Union{Float64,Int},y::Union{Float64,Int},w::Union{Float64,Int},h::Union{Float64,Int})=fill([x;x+w;x+w;x],[y;y;y+h;y+h])
draw_box(x::Union{Float64,Int},y::Union{Float64,Int},w::Union{Float64,Int},h::Union{Float64,Int},color::AbstractString)=fill([x;x+w;x+w;x],[y;y;y+h;y+h],color=color)

function draw_sedan(pomdp::MLPOMDP,x_ctr::Float64,y_ctr::Float64,color::AbstractString)
	#x_ctr = pomdp.POSITIONS[s.pos[1]]
	#y_ctr = pomdp.phys_param.y_interval*s.pos[2]
	w = pomdp.phys_param.l_car
	h = pomdp.phys_param.w_car
	x = x_ctr - 0.5*w
	y = y_ctr - 0.5*h

	##TODO: color stuff based on behavior Model
	draw_box(x,y,w,h,color)
	wind_color = ""
	x_windshield = x_ctr
	w_windshield = w*0.25
	h_windshield = h*0.75
	y_windshield = y_ctr - 0.5*h_windshield
	draw_box(x_windshield,y_windshield,w_windshield,h_windshield,window_color)
	x_backwindow = x_ctr - 0.25*w
	w_backwindow = w*0.25
	h_backwindow = w*0.75
	y_backwindow = y_ctr - *0.5*h_backwindow
	draw_box(x_backwindow,y_backwindow,w_backwindow,h_backwindow,window_color)
	##TODO: add direction arrow
	##TODO: add a face based on behavior model
end

function draw_sedan(pomdp::MLPOMDP,s::CarState)
	x_ctr = pomdp.POSITIONS[s.pos[1]]
	y_ctr = pomdp.phys_param.y_interval*s.pos[2]
	##TODO: something to do with behavior and color
	color = "" #PLACEHOLDER
	draw_sedan(pomdp,x_ctr,y_ctr,color)
	#draw_direction?
	#draw_ face?
end

function draw_crash(pomdp::MLPOMDP,s::MLState,a::MLAction)

end

function visualize(pomdp::MLPomdp,s::MLState,a::MLAction)
	#Placeholder!
	#make the initial canvas
	W = pomdp.phys_param.lane_length
	H = pomdp.phys_param.w_lane*(pomdp.nb_col+1)/2
	draw_box(0,0,W,H,"#424242") #a gray

	#draw hard borders on top and bottom
	border_thickness = 0.5
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

	#draw environment cars
	for car in s.env_cars
		draw_sedan(pomdp,car)
	end

	#if is_crash, add crash graphic
	if is_crash(pomdp,s,a)
		draw_crash(pomdp,s,a)
	end

end
