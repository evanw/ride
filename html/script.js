////////////////////////////////////////////////////////////////////////////////
// class LaunchFile
////////////////////////////////////////////////////////////////////////////////

var launchFileHTML = '\
<li class="divider"></li>\
<li class="nav-header"></li>\
<li><a>Show terminal output</a></li>\
<li><a>Stop and remove from list</a></li>\
';

function LaunchFile(name, id) {
  this.name = name;
  this.id = id;
  this.menuElements = $(launchFileHTML);
  this.menuElements[1].textContent = name;
  this.menuElements[2].firstChild.href = 'javascript:ui.showLaunchFile(' + JSON.stringify(id) + ')';
  this.menuElements[3].firstChild.href = 'javascript:ui.killLaunchFile(' + JSON.stringify(id) + ')';
}

////////////////////////////////////////////////////////////////////////////////
// RIDE
////////////////////////////////////////////////////////////////////////////////

var ride = {
  graph: new GraphBox.Graph(),
  launchFiles: {},

  reset: function() {
    this.graph.clear();
    this.launchFiles = {};
    $('#launch_files_menu').html('');
    $('#launch_files').hide();
  },

  deleteSelection: function() {
    // This will only work on owned nodes but will just do nothing for other nodes
    this.graph.selection().map(function(node) {
      ROS.call('/ride/node/destroy', { name: node.name });
    });
  },

  selectAll: function() {
    this.graph.setSelection(this.graph.nodes);
  },

  insertNode: function(package, file) {
    if (file.indexOf('.launch') == file.length - '.launch'.length) {
      ROS.call('/ride/launch/create', { package: package, launch_file: file });
    } else {
      ROS.call('/ride/node/create', { package: package, binary: file });
    }
  },

  update: function(data) {
    switch (data.type) {
      case 'create_node':
        var node = new GraphBox.Node(data.name);
        if (data.is_owned) {
          node.detailText = 'Starting...';
          node.displayName = data.display_name;
          node.readOnlyFlag = false;
        }
        this.graph.addNode(node);
        if (data.is_owned) ui.ownedNodeAdded(node);
        this.graph.updateBounds();
        this.graph.draw();
        break;

      case 'destroy_node':
        var node = this.graph.node(data.name);
        if (node) {
          this.graph.removeNode(node);
          this.graph.updateBounds();
          this.graph.draw();
        }
        break;

      case 'create_slot':
        var node = this.graph.node(data.node_name);
        if (!node) break;

        // Find the slot and list of slots
        if (data.is_input) {
          var slot = node.input(data.topic);
          var slots = node.inputs;
        } else {
          var slot = node.output(data.topic);
          var slots = node.outputs;
        }

        // Create the slot if it doesn't exist
        if (!slot) {
          slot = new GraphBox.Connection(data.topic);
          if (data.original_topic) {
            slot.displayName = data.original_topic;
            slot.readOnlyFlag = false;
          } else {
            slot.readOnlyFlag = true;
          }
          slots.push(slot);
          node.updateHTML();
          this.graph.updateBounds();
        }

        // Connect the slots to all other slots of the opposite type with the same name
        this.graph.nodes.map(function(node) {
          (data.is_input ? node.outputs : node.inputs).map(function(other) {
            if (other.name == slot.name) {
              slot.connect(other);
            }
          });
        });
        this.graph.draw();
        break;

      case 'destroy_slot':
        var node = this.graph.node(data.node_name);
        if (!node) break;

        // Find the slot and list of slots
        if (data.is_input) {
          var slot = node.input(data.topic);
          var slots = node.inputs;
        } else {
          var slot = node.output(data.topic);
          var slots = node.outputs;
        }

        // Remove the slot if it exists
        if (slot) {
          slots.splice(slots.indexOf(slot), 1);
          node.updateHTML();
          this.graph.updateBounds();
          this.graph.draw();
        }
        break;

      case 'create_link':
        var slots = this.getLinkSlots(data.from_topic, data.to_topic);
        slots.inputs.map(function(input) {
          slots.outputs.map(function(output) {
            input.connect(output);
          });
        });
        this.graph.draw();
        break;

      case 'destroy_link':
        var slots = this.getLinkSlots(data.from_topic, data.to_topic);
        slots.inputs.map(function(input) {
          slots.outputs.map(function(output) {
            input.disconnect(output);
          });
        });
        this.graph.draw();
        break;

      case 'update_owned_node':
        var node = this.graph.node(data.name);
        if (!node) break;
        node.isRunning = data.is_running;
        node.detailText = data.status;
        node.updateHTML();
        $(node.startElement).toggle(!data.is_running);
        $(node.stopElement).toggle(data.is_running);
        this.graph.updateBounds();
        this.graph.draw();
        if (!data.is_running && node.relaunchWhenStopped) {
          node.relaunchWhenStopped = false;
          ROS.call('/ride/node/start', { name: data.name });
        }
        break;

      case 'create_launch_file':
        var launchFile = new LaunchFile(data.name, data.id);
        this.launchFiles[launchFile.id] = launchFile;
        launchFile.menuElements.appendTo('#launch_files_menu');
        var count = $('#launch_files_menu .divider').length;
        $('#launch_files_count').html('Launch Files (' + count + ') <b class="caret"></b>');
        $('#launch_files').show();
        break;

      case 'destroy_launch_file':
        var launchFile = this.launchFiles[data.id];
        if (launchFile) {
          launchFile.menuElements.remove();
          delete this.launchFiles[data.id];
          var count = $('#launch_files_menu .divider').length;
          $('#launch_files_count').html('Launch Files (' + count + ') <b class="caret"></b>');
          if (!count) $('#launch_files').hide();
        }
        break;
    }
  },

  getLinkSlots: function(from_topic, to_topic) {
    var slots = { inputs: [], outputs: [] };
    this.graph.nodes.map(function(node) {
      node.inputs.map(function(input) {
        if (input.name == to_topic) slots.inputs.push(input);
      });
      node.outputs.map(function(output) {
        if (output.name == from_topic) slots.outputs.push(output);
      });
    });
    return slots;
  }
};

// Propagate new connections to the server
ride.graph.onconnection = function(input, output) {
  ROS.call('/ride/link/create', {
    from_topic: output.name,
    to_topic: input.name
  });
};

// Propagate new disconnections to the server
ride.graph.ondisconnection = function(input, output) {
  ROS.call('/ride/link/destroy', {
    from_topic: output.name,
    to_topic: input.name
  });
};

////////////////////////////////////////////////////////////////////////////////
// Network
////////////////////////////////////////////////////////////////////////////////

ROS.onopen = function() {
  ui.setConnected(true);

  // Make sure we are subscribed first before we start, otherwise it will be a
  // race condition and we might miss stuff. But ignore all updates until after
  // we start successfully since they don't make sense otherwise.
  var loaded = false;
  ROS.subscribe('/ride/updates', function(data) {
    if (loaded) {
      ride.update(JSON.parse(data.data));
    }
  });
  ROS.call('/ride/load', {}, function(data) {
    data = JSON.parse(data.json);
    data.updates.map(function(data) {
      ride.update(data);
    });
    ui.setPackages(data.packages);
    loaded = true;
  });
};

ROS.onclose = function() {
  ui.setConnected(false);
};

ROS.connect();

////////////////////////////////////////////////////////////////////////////////
// UI
////////////////////////////////////////////////////////////////////////////////

var dropdownHTML = '\
  <div class="dropdown">\
    <a data-toggle="dropdown"><div class="caret"></div></a>\
    <ul class="dropdown-menu"></ul>\
  </div>\
';

var ui = {
  ownedNodeAdded: function(node) {
    // Create dropdown menu
    function item(name, callback) {
      return $('<li><a>' + name + '</a></li>')
        .appendTo(menu).click(callback)[0];
    }
    $(dropdownHTML).insertAfter(node.titleElement);
    var menu = $(node.element).find('.dropdown-menu')[0];

    // Start node
    node.startElement = item('Start node', function() {
      ROS.call('/ride/node/start', { name: node.name });
    });
    $(node.startElement).hide();

    // Stop node
    node.stopElement = item('Stop node', function() {
      ROS.call('/ride/node/stop', { name: node.name });
    });
    $(node.stopElement).show();

    // Launch settings
    item('Launch settings', function() {
      ROS.call('/ride/node/settings/get', { name: node.name }, function(data) {
        $('#launch_settings_node_name').val(node.name);
        $('#launch_settings_cmd_line_args').val(data.cmd_line_args);
        $('#launch_settings_rosparams').val(data.rosparams);
        $('#launch_settings_env_vars').val(data.env_vars);
        $('#launch_settings').modal('show');
      });
    });

    // Show terminal output
    item('Show terminal output', function() {
      ROS.call('/ride/node/output', { name: node.name }, function(data) {
        $('#terminal_output_data').html(escape_codes_to_html(data.output));
        $('#terminal_output').modal('show');
      });
    });
  },

  changeConnectionURL: function() {
    $('#new_connection_url').val(ROS.url);
    $('#change_connection_url').modal('show');
  },

  setConnectionURL: function() {
    ROS.disconnect();
    ROS.url = $('#new_connection_url').val();
    ROS.connect();
    $('#change_connection_url').modal('hide');
  },

  setLaunchSettings: function(relaunch) {
    var node_name = $('#launch_settings_node_name').val();
    ROS.call('/ride/node/settings/set', {
      name: node_name,
      cmd_line_args: $('#launch_settings_cmd_line_args').val(),
      rosparams: $('#launch_settings_rosparams').val(),
      env_vars: $('#launch_settings_env_vars').val()
    }, function() {
      var node = ride.graph.node(node_name);
      if (!node) return;
      if (node.isRunning) {
        ROS.call('/ride/node/stop', { name: node_name });
        node.relaunchWhenStopped = true;
      } else {
        ROS.call('/ride/node/start', { name: node_name });
      }
    });
    $('#launch_settings').modal('hide');
  },

  setConnected: function(connected) {
    if (connected) {
      $('#connection_status').text('Connected to ' + ROS.url);
    } else {
      $('#connection_status').text('Trying to connect to ' + ROS.url);
    }
    ride.reset();
  },

  setPackages: function(packages) {
    var autocomplete = [];
    packages = packages;
    packages.map(function(package) {
      package.binaries.concat(package.launch_files).map(function(path) {
        path = path.substring(path.lastIndexOf('/') + 1);
        autocomplete.push(path + ' (' + package.name + ')');
      });
    });
    var insert_node = $('#insert_node');
    insert_node.typeahead({ source: autocomplete, items: 16 });
    insert_node.data('typeahead').select = function() {
      $.fn.typeahead.Constructor.prototype.select.call(this);
      ui.insertNode();
    };
  },

  insertNode: function() {
    // Right now this relies on the format 'fileName (packageName)' because
    // that's what is used in the autocomplete
    var insert_node = $('#insert_node');
    var name = insert_node.val();
    var match = /^([^ ]+) \(([^ ]+)\)$/.exec(name);
    if (match) {
      var packageName = match[2];
      var fileName = match[1];
      ride.insertNode(packageName, fileName);
      insert_node.val('');
      insert_node.blur();
    }
  },

  showLaunchFile: function(id) {
    ROS.call('/ride/launch/output', { id: id }, function(data) {
        $('#terminal_output_data').html(escape_codes_to_html(data.output));
        $('#terminal_output').modal('show');
    });
  },

  killLaunchFile: function(id) {
    ROS.call('/ride/launch/destroy', { id: id });
  }
};

// Update the connection status
ui.setConnected(false);

// Insert the graph where this script tag is in the DOM
document.body.appendChild(ride.graph.element);
ride.graph.updateBounds();
ride.graph.draw();

// Compute which input has focus
var inputWithFocus = null;
$('input, textarea').focus(function() { inputWithFocus = this; });
$('input, textarea').blur(function() { inputWithFocus = null; });

// Deselect input when the graph is clicked
$(ride.graph.element).click(function() {
  $('input').blur();
});

$(document).keydown(function(e) {
  // Delete the selection when backspace or delete is pressed
  if (!inputWithFocus && (e.which == 8 || e.which == 46)) {
    e.preventDefault();
    ride.deleteSelection();
  }

  // Ctrl+A selects all nodes
  if (!inputWithFocus && e.which == 65 && (e.ctrlKey || e.metaKey)) {
    e.preventDefault();
    ride.selectAll();
  }

  // Ctrl+I sets focus to the insert node textbox
  if (e.which == 73 && (e.ctrlKey || e.metaKey)) {
    e.preventDefault();
    $('#insert_node').focus();
  }
});

// Resize the graph when the window resizes
var navbarHeight = $('#navbar').height();
function resizeGraph() {
  $(ride.graph.element).css({
    position: 'absolute',
    left: 0,
    top: navbarHeight,
    width: window.innerWidth,
    height: window.innerHeight - navbarHeight
  });
  ride.graph.updateBounds();
  ride.graph.draw();
}
$(window).resize(resizeGraph);
resizeGraph();

////////////////////////////////////////////////////////////////////////////////
// Helper functions
////////////////////////////////////////////////////////////////////////////////

// Convert ASCII escape codes into colored HTML because why not
function escape_codes_to_html(text) {
  var colors = ['black', 'red', 'green', 'yellow', 'blue', 'magenta', 'cyan', 'white'];
  var html = text.replace(/&/g, '&amp;').replace(/</g, '&lt;').replace(/>/g, '&gt;');
  return '<span>' + html.replace(new RegExp('\x1b\\[\\d+(;\\d+)*[A-Za-z]', 'g'), function(match) {
    return match.substring(2, match.length - 1).split(';').map(function(code) {
      if (/^3\d$/.test(code)) return '</span><span style="color:' + colors[code[1]] + ';">';
      if (/^4\d$/.test(code)) return '</span><span style="background:' + colors[code[1]] + ';">';
      return '';
    }).join('');
  }) + '</span>';
}
