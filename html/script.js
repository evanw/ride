////////////////////////////////////////////////////////////////////////////////
// RIDE
////////////////////////////////////////////////////////////////////////////////

var STATUS_STARTING = 0;
var STATUS_STARTED = 1;
var STATUS_STOPPING = 2;
var STATUS_STOPPED = 3;

var ride = {
  graph: new GraphBox.Graph(),

  reset: function() {
    this.graph.clear();
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

  insertNode: function(package, binary) {
    ROS.call('/ride/node/create', { package: package, binary: binary }, function(data) {
      ROS.call('/ride/node/start', { name: data.name });
    });
  },

  update: function(data) {
    switch (data.type) {
      case 'create_node':
        var node = new GraphBox.Node(data.name);
        node.readOnlyFlag = !data.is_owned;
        this.graph.addNode(node);
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

var ui = {
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
      package.binaries.map(function(binary) {
        binary = binary.substring(binary.lastIndexOf('/') + 1);
        autocomplete.push(binary + ' (' + package.name + ')');
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
    // Right now this relies on the format 'binaryName (packageName)' because
    // that's what is used in the autocomplete
    var insert_node = $('#insert_node');
    var name = insert_node.val();
    var match = /^([^ ]+) \(([^ ]+)\)$/.exec(name);
    if (match) {
      var packageName = match[2];
      var binaryName = match[1];
      ride.insertNode(packageName, binaryName);
      insert_node.val('');
      insert_node.blur();
    }
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
$('input').focus(function() { inputWithFocus = this; });
$('input').blur(function() { inputWithFocus = null; });

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
