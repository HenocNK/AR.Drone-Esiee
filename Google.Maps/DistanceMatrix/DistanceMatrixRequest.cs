﻿/*
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.
 * The ASF licenses this file to You under the Apache License, Version 2.0
 * (the "License"); you may not use this file except in compliance with
 * the License.  You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

using System;
using System.Collections.Generic;
using System.ComponentModel;
using Google.Maps;
using System.Text;
using System.Linq;

namespace Google.Maps.DistanceMatrix
{
	/// <summary>
	/// Provides a request for the Google Distance Matrix web service.
	/// </summary>
	public class DistanceMatrixRequest : BaseRequest
	{
		/// <summary>
		/// (optional) Specifies what mode of transport to use when calculating directions.
		/// </summary>
		public TravelMode Mode { get; set; }

		/// <summary>
		/// (optional) Directions may be calculated that adhere to certain restrictions.
		/// </summary>
		[DefaultValue(Avoid.none)]
		public Avoid Avoid { get; set; }

		/// <summary>
		///  (optional) Specifies the unit system to use when expressing distance as text.
		///   <see href="https://developers.google.com/maps/documentation/distance-matrix/intro#unit_systems"/>
		/// </summary>
		public Units Units { get; set; }

		/// <summary>
		/// (optional) The language in which to return results.
		/// <see href="http://code.google.com/apis/maps/faq.html#languagesupport" />
		/// </summary>
		public string Language { get; set; }

		/// <summary>
		/// (optional) Specifies the desired time of arrival for transit requests, in seconds since midnight, January 1, 1970 UTC.
		/// You can specify either departure_time or arrival_time, but not both.
		/// Note that arrival_time must be specified as an integer.
		/// <see href="https://developers.google.com/maps/documentation/distance-matrix/intro" />
		/// </summary>
		public int? DepartureTime { get; set; }

		/// <summary>
		/// (optional) The desired time of departure. You can specify the time as an integer in seconds since midnight, January 1, 1970 UTC.
		/// Alternatively, you can specify a value of now, which sets the departure time to the current time (correct to the nearest second).
		/// <see href="https://developers.google.com/maps/documentation/distance-matrix/intro" />
		/// </summary>
		public int? ArrivalTime { get; set; }

		/// <summary>
		/// (optional) Specifies the assumptions to use when calculating time in traffic.
		/// This setting affects the value returned in the duration_in_traffic field in the response, which contains the predicted time in traffic based on historical averages.
		/// The traffic_model parameter may only be specified for requests where the travel mode is driving, and where the request includes a departure_time, and only if the request includes an API key or a Google Maps Platform Premium Plan client ID
		/// <see href="https://developers.google.com/maps/documentation/distance-matrix/intro" />
		/// </summary>
		public TrafficModels TrafficModel { get; set; }

		/// <summary>
		/// (optional) Specifies one or more preferred modes of transit. This parameter may only be specified for requests where the mode is transit
		/// <see href="https://developers.google.com/maps/documentation/distance-matrix/intro" />
		/// </summary>
		public TransitModes TransitMode { get; set; }

		/// <summary>
		/// (optional) Specifies preferences for transit requests. Using this parameter, you can bias the options returned, rather than accepting the default best route chosen by the API.
		/// This parameter may only be specified for requests where the mode is transit.
		/// <see href="https://developers.google.com/maps/documentation/distance-matrix/intro" />
		/// </summary>
		public TransitRoutingPreferences TransitRoutingPreference { get; set; }

		/// <summary>
		///  List of origin waypoints
		/// </summary>
		private List<Location> _waypointsOrigin;
		private List<Location> EnsureWaypointsOrigin()
		{
			if(_waypointsOrigin == null)
			{
				_waypointsOrigin = new List<Location>();
			}
			return _waypointsOrigin;
		}

		/// <summary>
		/// List of destination waypoints
		/// </summary>
		private List<Location> _waypointsDestination;
		private List<Location> EnsureWaypointsDestination()
		{
			if(_waypointsDestination == null)
			{
				_waypointsDestination = new List<Location>();
			}
			return _waypointsDestination;
		}

		/// <summary>
		/// Accessor method
		/// </summary>
		public List<Location> WaypointsOrigin
		{
			get
			{
				return EnsureWaypointsOrigin();
			}
			set
			{
				_waypointsOrigin = value;
			}
		}//end method

		/// <summary>
		/// Accessor method
		/// </summary>
		public List<Location> WaypointsDestination
		{
			get
			{
				return EnsureWaypointsDestination();
			}
			set
			{
				_waypointsDestination = value;
			}
		}//end method

		/// <summary>
		/// Adds a waypoint location to the origin set
		/// </summary>
		/// <param name="destination"></param>
		public void AddOrigin(Location destination)
		{
			WaypointsOrigin.Add(destination);
		}

		/// <summary>
		/// Adds a waypoint location to the destination set
		/// </summary>
		/// <param name="destination"></param>
		public void AddDestination(Location destination)
		{
			WaypointsDestination.Add(destination);
		}

		/// <summary>
		/// Convert waypoint locations collection to a uri string
		/// </summary>
		/// <returns></returns>
		internal string WaypointsToUri(IEnumerable<Location> waypointsList)
		{
			if(waypointsList == null) return string.Empty;
			if(waypointsList.Count() == 0) return string.Empty;

			StringBuilder sb = new StringBuilder();

			foreach(Location waypoint in waypointsList)
			{
				if(sb.Length > 0) sb.Append("|");
				sb.Append(Uri.EscapeDataString(waypoint.ToString()));
			}

			return sb.ToString();
		}

		/// <summary>
		/// Create URI for quering
		/// </summary>
		/// <returns></returns>
		public override Uri ToUri()
		{
			var qsb = new Internal.QueryStringBuilder()
				.Append("origins", WaypointsToUri(_waypointsOrigin))
				.Append("destinations", WaypointsToUri(_waypointsDestination))
				.Append("mode", Mode.ToString())
				.Append("language", Language)
				.Append("units", Units.ToString())
				.Append("avoid", AvoidHelper.MakeAvoidString(Avoid))
				.Append("departure_time", DepartureTime.ToString());

			if(DepartureTime == null)
				qsb.Append("arrival_time", ArrivalTime.ToString());

			if (DepartureTime != null && Mode.Equals(TravelMode.driving))
				qsb.Append("traffic_model ", TrafficModel.ToString());

			if (Mode.Equals(TravelMode.transit))
			{
				qsb.Append("transit_mode", TransitMode.ToString());
				qsb.Append("transit_routing_preference ", TransitRoutingPreference.ToString());
			}

			var url = "json?" + qsb.ToString();

			return new Uri(url, UriKind.Relative);
		}
	}

}
