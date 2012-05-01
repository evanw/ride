var GraphLayout = (function() {
  "use strict";

  //////////////////////////////////////////////////////////////////////////////
  // class Map
  //
  // A map that is safe to use for keys like "prototype", unlike object literals
  //////////////////////////////////////////////////////////////////////////////

  function Map() {
    this.items = {};
  }

  Map.prototype = {
    add: function(key, value) { this.items['$$' + key] = value; },
    remove: function(key) { delete this.items['$$' + key]; },
    find: function(key) { return this.items['$$' + key] || null; },
    contains: function(key) { return ('$$' + key) in this.items; },
    all: function() {
      return Object.keys(this.items).map(function(key) {
        return this.items[key];
      }, this);
    }
  };

  //////////////////////////////////////////////////////////////////////////////
  // class Node
  //////////////////////////////////////////////////////////////////////////////

  function Node(name, width, height) {
    this.name = name;
    this.width = width;
    this.height = height;
    this.x = null;
    this.y = null;
    this.inputs = new Map();
    this.outputs = new Map();
  }

  Node.prototype = {
    connectTo: function(node) {
      this.outputs.add(node.name, node);
      node.inputs.add(this.name, this);
    },

    isConnectedTo: function(node) {
      return this.outputs.contains(node.name);
    },

    disconnectFrom: function(node) {
      this.outputs.remove(node.name);
      node.inputs.remove(this.name);
    }
  };

  //////////////////////////////////////////////////////////////////////////////
  // class Graph
  //////////////////////////////////////////////////////////////////////////////

  function Graph() {
    this.nodes = new Map();
  }

  Graph.prototype = {
    addNode: function(node) {
      this.nodes.add(node.name, node);
    },

    node: function(name) {
      return this.nodes.find(name);
    },

    layout: function(paddingBig, paddingSmall) {
      // Force a topological order on the graph using DFS, ignoring cycles
      function dfs(node, x, y) {
        var height = 0;
        if (visited.contains(node.name)) return;
        visited.add(node.name, node);
        node.outputs.all().map(function(output) {
          if (!visited.contains(output.name)) {
            if (height) height += paddingSmall;
            height += dfs(output, x + node.width + paddingBig, y + height);
          }
        });
        height = Math.max(node.height, height);
        node.x = x;
        node.y = y + (height - node.height) / 2;
        return height;
      }

      // Start the layout in the top left
      var x = paddingSmall, y = paddingSmall;
      var visited = new Map();
      var all = this.nodes.all();

      // Visit root nodes before other nodes (still need to visit all nodes to
      // be sure they are all visited in the case of cycles)
      all.filter(function(n) { return !n.inputs.all().length; })
        .concat(all.filter(function(n) { return n.inputs.all().length; }))
        .map(function(n) { y += dfs(n, x, y) + paddingSmall; });
    }
  };

  return { Graph: Graph, Node: Node };

})();
