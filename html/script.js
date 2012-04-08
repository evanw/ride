////////////////////////////////////////////////////////////////////////////////
// RIDE
////////////////////////////////////////////////////////////////////////////////

var ride = {
  graph: new GraphBox.Graph(),

  addNode: function(packageName, nodeName) {
    var node = new GraphBox.Node(nodeName + ' (' + packageName + ')');
    node.detailText = 'Loading...';
    node.moveTo(100, 100);
    this.graph.addNode(node);
  },

  deleteSelection: function() {
    this.graph.selection().map(function(node) {
      this.graph.removeNode(node);
    }, this);
    this.graph.updateBounds();
  },

  updateNodeList: function(data) {
    // Create new nodes and remove old nodes
    var existingNodes = {};
    this.graph.nodes.map(function(node) {
      existingNodes[node.name] = node;
    });
    data.map(function(info) {
      if (!(info.name in existingNodes)) {
        ride.graph.addNode(new GraphBox.Node(info.name));
      } else {
        delete existingNodes[info.name];
      }
    });
    Object.keys(existingNodes).map(function(name) {
      ride.graph.removeNode(existingNodes[name]);
    });

    // Create new connections and remove old connections
    data.map(function(info) {
      var node = ride.graph.node(info.name);
      info.subscribed.map(function(topic) {
        if (!node.input(topic)) {
          node.inputs.push(new GraphBox.Connection(topic));
        }
      });
      info.published.map(function(topic) {
        if (!node.output(topic)) {
          node.outputs.push(new GraphBox.Connection(topic));
        }
      });
      node.inputs = node.inputs.filter(function(input) {
        return info.subscribed.indexOf(input.name) != -1;
      });
      node.outputs = node.outputs.filter(function(output) {
        return info.published.indexOf(output.name) != -1;
      });
    });

    // Update connection targets
    var subscribers = {};
    var publishers = {};
    data.map(function(info) {
      var node = ride.graph.node(info.name);
      info.subscribed.map(function(topic) {
        (subscribers[topic] || (subscribers[topic] = [])).push(node);
      });
      info.published.map(function(topic) {
        (publishers[topic] || (publishers[topic] = [])).push(node);
      });
    });
    this.graph.nodes.map(function(node) {
      node.inputs.map(function(input) {
        input.targets = (publishers[input.name] || []).map(function(node) {
          return node.output(input.name);
        });
      });
      node.outputs.map(function(output) {
        output.targets = (subscribers[output.name] || []).map(function(node) {
          return node.input(output.name);
        });
      });
    });

    // Update the graph
    this.graph.nodes.map(function(node) {
      node.updateHTML();
    });
    this.graph.updateBounds();
  }
};

////////////////////////////////////////////////////////////////////////////////
// Network
////////////////////////////////////////////////////////////////////////////////

ROS.onopen = function() {
  ui.setConnected(true);
  ROS.call('/ride/package/list', {}, function(data) {
    ui.setPackages(data.packages);
  });
};

ROS.onclose = function() {
  ui.setConnected(false);
};

setInterval(function() {
  ROS.call('/ride/node/list', {}, function(data) {
    ride.updateNodeList(data.nodes);
  });
}, 500);

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
    // Right now this relies on the format 'nodeName (packageName)' because
    // that's what is used in the autocomplete
    var insert_node = $('#insert_node');
    var name = insert_node.val();
    var match = /^([^ ]+) \(([^ ]+)\)$/.exec(name);
    if (match) {
      var packageName = match[2];
      var nodeName = match[1];
      ride.addNode(packageName, nodeName);
      ride.graph.setSelection([ride.graph.nodes[ride.graph.nodes.length - 1]]);
      insert_node.val('');
      insert_node.blur();
    }
  },

  setConnected: function(connected) {
    if (connected) {
      $('#connection_status').text('Connected to ' + ROS.url);
    } else {
      $('#connection_status').text('Trying to connect to ' + ROS.url);
    }
  }
};

// Update the connection status
ui.setConnected(false);

// Insert the graph where this script tag is in the DOM
document.body.appendChild(ride.graph.element);
ride.graph.updateBounds();

// Compute which input has focus
var inputWithFocus = null;
$('input').focus(function() { inputWithFocus = this; });
$('input').blur(function() { inputWithFocus = null; });

// Delete the selection when backspace or delete is pressed
$(document).keydown(function(e) {
  if (!inputWithFocus && (e.which == 8 || e.which == 46)) {
    e.preventDefault();
    ride.deleteSelection();
  }
});

// Ctrl+I sets focus to the insert node textbox
$(document).keydown(function(e) {
  if (e.which == 73 && e.ctrlKey) {
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
}
$(window).resize(resizeGraph);
resizeGraph();
