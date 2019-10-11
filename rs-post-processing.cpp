// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved.


#include "librealsense2/rs.hpp" // Include RealSense Cross Platform API
#include "example.hpp"          // Include short list of convenience functions for rendering

#include <map>
#include <string>
#include <thread>
#include <atomic>

#include <imgui.h>
#include <imgui_impl_glfw.h>

struct state { double yaw, pitch, last_x, last_y; bool ml; float offset_x, offset_y; texture tex; };


float get_depth_scale(rs2::device dev);
rs2_stream find_stream_to_align(const std::vector<rs2::stream_profile>& streams);
bool profile_changed(const std::vector<rs2::stream_profile>& current, const std::vector<rs2::stream_profile>& prev);

/**
Helper class for controlling the filter's GUI element
*/
struct filter_slider_ui
{

	std::string name;
	std::string label;
	std::string description;
	bool is_int;
	float value;
	rs2::option_range range;

	bool render(const float3& location, bool enabled);
	static bool is_all_integers(const rs2::option_range& range);
};

/**
Class to encapsulate a filter alongside its options
*/
class filter_options
{
public:
	filter_options(const std::string name, rs2::filter& filter);
	filter_options(filter_options&& other);
	std::string filter_name;                                   //Friendly name of the filter
	rs2::filter& filter;                                       //The filter in use
	std::map<rs2_option, filter_slider_ui> supported_options;  //maps from an option supported by the filter, to the corresponding slider
	std::atomic_bool is_enabled;                               //A boolean controlled by the user that determines whether to apply the filter or not
};

// Helper functions for rendering the UI
void render_ui(float w, float h, std::vector<filter_options>& filters);
// Helper function for getting data from the queues and updating the view
void update_data2(rs2::frame_queue& data, rs2::frame& depth, rs2::points& points, rs2::pointcloud& pc, glfw_state& view, rs2::colorizer& color_map, rs2::align& align, rs2_stream& align_to, rs2::frameset& processed);

int main(int argc, char * argv[]) try
{
	// Create a simple OpenGL window for rendering:
	window app(1000, 1000, "RealSense Post Processing Example");
	ImGui_ImplGlfw_Init(app, false);

	// Construct objects to manage view state
	glfw_state filtered_view_orientation{};

	// register callbacks to allow manipulation of the pointcloud
	register_glfw_callbacks(app, filtered_view_orientation);

	// Declare pointcloud objects, for calculating pointclouds and texture mappings
	rs2::pointcloud filtered_pc;



	// Create a pipeline to easily configure and start the camera
	rs2::pipeline pipe;
	//Calling pipeline's start() without any additional parameters will start the first device
	// with its default streams.
	//The start function returns the pipeline profile which the pipeline used to start the device
	rs2::pipeline_profile profile = pipe.start();

	// Each depth camera might have different units for depth pixels, so we get it here
	// Using the pipeline's profile, we can retrieve the device that the pipeline uses
	float depth_scale = get_depth_scale(profile.get_device());

	//Pipeline could choose a device that does not have a color stream
	//If there is no color stream, choose to align depth to another stream
	rs2_stream align_to = find_stream_to_align(profile.get_streams());
	rs2::frameset processed;//AA declaring frameset

	// Create a rs2::align object.
	// rs2::align allows us to perform alignment of depth frames to others frames
	//The "align_to" is the stream type to which we plan to align depth frames.
	rs2::align align(align_to);

	// Declare filters
	rs2::decimation_filter dec_filter;  // Decimation - reduces depth frame density
	rs2::threshold_filter thr_filter;   // Threshold  - removes values outside recommended range
	rs2::spatial_filter spat_filter;    // Spatial    - edge-preserving spatial smoothing
	rs2::temporal_filter temp_filter;   // Temporal   - reduces temporal noise

										// Declare disparity transform from depth to disparity and vice versa
	const std::string disparity_filter_name = "Disparity";
	rs2::disparity_transform depth_to_disparity(true);
	rs2::disparity_transform disparity_to_depth(false);

	// Initialize a vector that holds filters and their options
	std::vector<filter_options> filters;

	// The following order of emplacement will dictate the orders in which filters are applied
	filters.emplace_back("Decimate", dec_filter);
	filters.emplace_back("Threshold", thr_filter);
	filters.emplace_back(disparity_filter_name, depth_to_disparity);
	filters.emplace_back("Spatial", spat_filter);
	filters.emplace_back("Temporal", temp_filter);

	// Declaring two concurrent queues that will be used to enqueue and dequeue frames from different threads
	rs2::frame_queue filtered_data;

	// Declare depth colorizer for pretty visualization of depth data
	rs2::colorizer color_map;

	// Atomic boolean to allow thread safe way to stop the thread
	std::atomic_bool stopped(false);

	// Create a thread for getting frames from the device and process them; this will also be used for the aligning now
	// to prevent UI thread from blocking due to long computations.



	std::thread processing_thread([&]() {
		while (!stopped) //While application is running
		{
			rs2::frameset data = pipe.wait_for_frames(); // Wait for next set of frames from the camera

			// rs2::pipeline::wait_for_frames() can replace the device it uses in case of device error or disconnection.
			// Since rs2::align is aligning depth to some other stream, we need to make sure that the stream was not changed
			//  after the call to wait_for_frames();
			if (profile_changed(pipe.get_active_profile().get_streams(), profile.get_streams()))
			{
				//If the profile was changed, update the align object, and also get the new device's depth scale
				profile = pipe.get_active_profile();
				align_to = find_stream_to_align(profile.get_streams());
				align = rs2::align(align_to);
				depth_scale = get_depth_scale(profile.get_device());
			}

			//Get processed aligned frame
			processed = align.process(data);

			// Trying to get both other and aligned depth frames
			rs2::video_frame other_frame = processed.first(align_to);
			rs2::depth_frame aligned_depth_frame = processed.get_depth_frame();

			//If one of them is unavailable, continue iteration
			if (!aligned_depth_frame || !other_frame)
			{
				continue;
			}


			//if (!depth_frame) // Should not happen but if the pipeline is configured differently
			//    return;       //  it might not provide depth and we don't want to crash

			rs2::frame filtered = aligned_depth_frame; // Does not copy the frame, only adds a reference


			/* Apply filters.
			The implemented flow of the filters pipeline is in the following order:
			1. apply decimation filter
			2. apply threshold filter
			3. transform the scene into disparity domain
			4. apply spatial filter
			5. apply temporal filter
			6. revert the results back (if step Disparity filter was applied
			to depth domain (each post processing block is optional and can be applied independantly).
			*/
			bool revert_disparity = false;
			for (auto&& filter : filters)
			{
				if (filter.is_enabled)
				{
					filtered = filter.filter.process(filtered);
					if (filter.filter_name == disparity_filter_name)
					{
						revert_disparity = true;
					}
				}
			}
			if (revert_disparity)
			{
				filtered = disparity_to_depth.process(filtered);
			}

			// Push filtered & original data to their respective queues
			// Note, pushing to two different queues might cause the application to display
			//  original and filtered pointclouds from different depth frames
			//  To make sure they are synchronized you need to push them together or add some
			//  synchronization mechanisms
			filtered_data.enqueue(filtered);
		}
	});


	// Declare objects that will hold the calculated pointclouds and colored frames
	// We save the last set of data to minimize flickering of the view
	rs2::frame colored_depth;
	rs2::frame colored_filtered;

	rs2::points filtered_points;


	// Save the time of last frame's arrival
	auto last_time = std::chrono::high_resolution_clock::now();
	

	while (app)
	{
		float w = static_cast<float>(app.width());
		float h = static_cast<float>(app.height());

		// Render the GUI
		render_ui(w, h, filters);

		// Try to get new data from the queues and update the view with new texture
	


		update_data2(filtered_data, colored_filtered, filtered_points, filtered_pc, filtered_view_orientation, color_map, align, align_to, processed);


		

		

		// Draw the pointclouds of the filtered frame (if the are available already)
		
		if (colored_filtered && filtered_points)
		{
			glViewport(0, int(h) / 2, int(w), int(h) / 2);
			draw_pointcloud(int(w)/2, int(h) / 2, filtered_view_orientation, filtered_points);
		}
		// Update time of current frame's arrival
		auto curr = std::chrono::high_resolution_clock::now();
		
	}

	// Signal the processing thread to stop, and join
	// (Not the safest way to join a thread, please wrap your threads in some RAII manner)
	stopped = true;
	processing_thread.join();

	return EXIT_SUCCESS;
}
catch (const rs2::error & e)
{
	std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
	return EXIT_FAILURE;
}
catch (const std::exception& e)
{
	std::cerr << e.what() << std::endl;
	return EXIT_FAILURE;
}

float get_depth_scale(rs2::device dev)
{
	// Go over the device's sensors
	for (rs2::sensor& sensor : dev.query_sensors())
	{
		// Check if the sensor if a depth sensor
		if (rs2::depth_sensor dpt = sensor.as<rs2::depth_sensor>())
		{
			return dpt.get_depth_scale();
		}
	}
	throw std::runtime_error("Device does not have a depth sensor");
}

rs2_stream find_stream_to_align(const std::vector<rs2::stream_profile>& streams)
{
	//Given a vector of streams, we try to find a depth stream and another stream to align depth with.
	//We prioritize color streams to make the view look better.
	//If color is not available, we take another stream that (other than depth)
	rs2_stream align_to = RS2_STREAM_ANY;
	bool depth_stream_found = false;
	bool color_stream_found = false;
	for (rs2::stream_profile sp : streams)
	{
		rs2_stream profile_stream = sp.stream_type();
		if (profile_stream != RS2_STREAM_DEPTH)
		{
			if (!color_stream_found)         //Prefer color
				align_to = profile_stream;

			if (profile_stream == RS2_STREAM_COLOR)
			{
				color_stream_found = true;
			}
		}
		else
		{
			depth_stream_found = true;
		}
	}

	if (!depth_stream_found)
		throw std::runtime_error("No Depth stream available");

	if (align_to == RS2_STREAM_ANY)
		throw std::runtime_error("No stream found to align with Depth");

	return align_to;
}

bool profile_changed(const std::vector<rs2::stream_profile>& current, const std::vector<rs2::stream_profile>& prev)
{
	for (auto&& sp : prev)
	{
		//If previous profile is in current (maybe just added another)
		auto itr = std::find_if(std::begin(current), std::end(current), [&sp](const rs2::stream_profile& current_sp) { return sp.unique_id() == current_sp.unique_id(); });
		if (itr == std::end(current)) //If it previous stream wasn't found in current
		{
			return true;
		}
	}
	return false;
}

void update_data(rs2::frame_queue& data, rs2::frame& colorized_depth, rs2::points& points, rs2::pointcloud& pc, glfw_state& view, rs2::colorizer& color_map)
{
	rs2::frame f;
	if (data.poll_for_frame(&f))  // Try to take the depth and points from the queue
	{
		points = pc.calculate(f); // Generate pointcloud from the depth data
		colorized_depth = color_map.process(f);     // Colorize the depth frame with a color map
		pc.map_to(colorized_depth);         // Map the colored depth to the point cloud
		pc.map_to(colorized_depth);
		view.tex.upload(colorized_depth);   //  and upload the texture to the view (without this the view will be B&W)
	}
}

void update_data2(rs2::frame_queue& data, rs2::frame& colorized_depth, rs2::points& points, rs2::pointcloud& pc, glfw_state& view, rs2::colorizer& color_map, rs2::align& align, rs2_stream& align_to, rs2::frameset& processed)
{
	rs2::frame f;
	if (data.poll_for_frame(&f))  // Try to take the depth and points from the queue
	{
		points = pc.calculate(f); // Generate pointcloud from the depth data
		colorized_depth = color_map.process(f);     // Colorize the depth frame with a color map
		rs2::video_frame other_frame = processed.first(align_to);
		pc.map_to(other_frame);
		view.tex.upload(other_frame);   //  and upload the texture to the view (without this the view will be B&W)
	}
}

void render_ui(float w, float h, std::vector<filter_options>& filters)
{
	// Flags for displaying ImGui window
	static const int flags = ImGuiWindowFlags_NoCollapse
		| ImGuiWindowFlags_NoScrollbar
		| ImGuiWindowFlags_NoSavedSettings
		| ImGuiWindowFlags_NoTitleBar
		| ImGuiWindowFlags_NoResize
		| ImGuiWindowFlags_NoMove;

	ImGui_ImplGlfw_NewFrame(1);
	ImGui::SetNextWindowSize({ w, h });
	ImGui::Begin("app", nullptr, flags);

	// Using ImGui library to provide slide controllers for adjusting the filter options
	const float offset_x = w / 4;
	const int offset_from_checkbox = 120;
	float offset_y = h / 2;
	float elements_margin = 45;
	for (auto& filter : filters)
	{
		// Draw a checkbox per filter to toggle if it should be applied
		ImGui::SetCursorPos({ offset_x, offset_y });
		ImGui::PushStyleColor(ImGuiCol_CheckMark, { 40 / 255.f, 170 / 255.f, 90 / 255.f, 1 });
		bool tmp_value = filter.is_enabled;
		ImGui::Checkbox(filter.filter_name.c_str(), &tmp_value);
		filter.is_enabled = tmp_value;
		ImGui::PopStyleColor();

		if (filter.supported_options.size() == 0)
		{
			offset_y += elements_margin;
		}
		// Draw a slider for each of the filter's options
		for (auto& option_slider_pair : filter.supported_options)
		{
			filter_slider_ui& slider = option_slider_pair.second;
			if (slider.render({ offset_x + offset_from_checkbox, offset_y, w / 4 }, filter.is_enabled))
			{
				filter.filter.set_option(option_slider_pair.first, slider.value);
			}
			offset_y += elements_margin;
		}
	}

	ImGui::End();
	ImGui::Render();
}

bool filter_slider_ui::render(const float3& location, bool enabled)
{
	bool value_changed = false;
	ImGui::PushStyleVar(ImGuiStyleVar_FrameRounding, 12);
	ImGui::PushStyleColor(ImGuiCol_SliderGrab, { 40 / 255.f, 170 / 255.f, 90 / 255.f, 1 });
	ImGui::PushStyleColor(ImGuiCol_SliderGrabActive, { 20 / 255.f, 150 / 255.f, 70 / 255.f, 1 });
	ImGui::GetStyle().GrabRounding = 12;
	if (!enabled)
	{
		ImGui::PushStyleColor(ImGuiCol_SliderGrab, { 0,0,0,0 });
		ImGui::PushStyleColor(ImGuiCol_SliderGrabActive, { 0,0,0,0 });
		ImGui::PushStyleColor(ImGuiCol_Text, { 0.6f, 0.6f, 0.6f, 1 });
	}

	ImGui::PushItemWidth(location.z);
	ImGui::SetCursorPos({ location.x, location.y + 3 });
	ImGui::TextUnformatted(label.c_str());
	if (ImGui::IsItemHovered())
		ImGui::SetTooltip("%s", description.c_str());

	ImGui::SetCursorPos({ location.x + 170, location.y });

	if (is_int)
	{
		int value_as_int = static_cast<int>(value);
		value_changed = ImGui::SliderInt(("##" + name).c_str(), &value_as_int, static_cast<int>(range.min), static_cast<int>(range.max), "%.0f");
		value = static_cast<float>(value_as_int);
	}
	else
	{
		value_changed = ImGui::SliderFloat(("##" + name).c_str(), &value, range.min, range.max, "%.3f", 1.0f);
	}

	ImGui::PopItemWidth();

	if (!enabled)
	{
		ImGui::PopStyleColor(3);
	}
	ImGui::PopStyleVar();
	ImGui::PopStyleColor(2);
	return value_changed;
}

/**
  Helper function for deciding on int ot float slider
*/
bool filter_slider_ui::is_all_integers(const rs2::option_range& range)
{
	const auto is_integer = [](float f)
	{
		return (fabs(fmod(f, 1)) < std::numeric_limits<float>::min());
	};

	return is_integer(range.min) && is_integer(range.max) &&
		is_integer(range.def) && is_integer(range.step);
}

/**
Constructor for filter_options, takes a name and a filter.
*/
filter_options::filter_options(const std::string name, rs2::filter& flt) :
	filter_name(name),
	filter(flt),
	is_enabled(true)
{
	const std::array<rs2_option, 5> possible_filter_options = {
		RS2_OPTION_FILTER_MAGNITUDE,
		RS2_OPTION_FILTER_SMOOTH_ALPHA,
		RS2_OPTION_MIN_DISTANCE,
		RS2_OPTION_MAX_DISTANCE,
		RS2_OPTION_FILTER_SMOOTH_DELTA
	};

	//Go over each filter option and create a slider for it
	for (rs2_option opt : possible_filter_options)
	{
		if (flt.supports(opt))
		{
			rs2::option_range range = flt.get_option_range(opt);
			supported_options[opt].range = range;
			supported_options[opt].value = range.def;
			supported_options[opt].is_int = filter_slider_ui::is_all_integers(range);
			supported_options[opt].description = flt.get_option_description(opt);
			std::string opt_name = flt.get_option_name(opt);
			supported_options[opt].name = name + "_" + opt_name;
			std::string prefix = "Filter ";
			supported_options[opt].label = opt_name;
		}
	}
}

filter_options::filter_options(filter_options&& other) :
	filter_name(std::move(other.filter_name)),
	filter(other.filter),
	supported_options(std::move(other.supported_options)),
	is_enabled(other.is_enabled.load())
{
}
