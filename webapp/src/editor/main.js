var c;
var editor;

function resizeCanvas() {
	var minSize = editor.getMinSize();
	c.canvas.width = minSize.width;
	c.canvas.height = minSize.height;
}

function draw() {
	resizeCanvas();
	editor.draw(c);
}

$(window).load(function() {
	c = $('#canvas')[0].getContext('2d');
	editor = new Editor();

	var nodes = editor.doc.nodes;
	nodes.push(new Node('gscam', [], ['image']));
	nodes.push(new Node('cmvision', ['image'], ['blobs']));
	nodes.push(new Node('ar_recog', ['image'], ['tags']));
	nodes.push(new Node('merge_blobs', ['blob_list'], ['blob_list']));
	nodes.push(new Node('filter_blobs', ['blob_list'], ['orange_over_green', 'green_over_orange', 'orange', 'green', 'yellow', 'pink']));
	nodes.push(new Node('seek_objects', ['blob_list'], ['twist']));
	nodes.push(new Node('avoid_objects', ['blob_list'], ['twist']));
	nodes.push(new Node('irobot_create_2_1', ['twist'], []));
	for (var i = 0; i < nodes.length; i++) {
		nodes[i].generateHTML();
	}

	draw();

	$(document).mousedown(function(e) {
		editor.mousePressed(e.pageX, e.pageY);
		e.preventDefault();
		draw();
	});

	$(document).mousemove(function(e) {
		editor.mouseMoved(e.pageX, e.pageY);
		e.preventDefault();
		draw();
	});

	$(document).mouseup(function(e) {
		editor.mouseReleased(e.pageX, e.pageY);
		e.preventDefault();
		draw();
	});

	$(document).keydown(function(e) {
		if ((e.ctrlKey || e.metaKey) && e.which == 'A'.charCodeAt(0)) {
			editor.selectAll();
			e.preventDefault();
			draw();
		}
	});

	$(window).resize(draw);
});
