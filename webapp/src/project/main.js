$(document).ready(function() {
	var contents = [
	new Toolbar.Button('Settings', '/static/images/settings.png').floatRight(),
		new Toolbar.Button('Run', '/static/images/run.png'),
		new Toolbar.Button('Stop', '/static/images/stop.png').setEnabled(false)
	];

	var toolbar = new Toolbar();
	toolbar.setContents(contents);

	function resize() {
		$('.editor').css({ top: toolbar.height() + 'px' });
	}
	$(window).resize(resize);
	$(window).load(resize);
	resize();
});
