# eufs_rviz_plugins
A package providing EUFS specific plugins for RViz. 

## Contributing
Many of the plugins in the
[`rviz_default_plugins`](https://github.com/ros2/rviz/tree/ros2/rviz_default_plugins) repository 
can be used as inspiration to make specific plugins.

## Plugins

### Displays
- #### ConeArrayWithCovarianceDisplay
  This plugin provides a visualisation of the `eufs_msg::msg::ConeArrayWithCovariance` message. It 
allows visualisations of cones and their covariance. However, in RViz, the cone and covariance 
visuals can individually be toggled under the namespace property in the display panel.

- #### WaypointArrayStampedDisplay
  This plugin provides a visualisation for the `eufs_msg::msg::WaypointArrayStamped` message. 

### Tools

### View Controllers


### Panels