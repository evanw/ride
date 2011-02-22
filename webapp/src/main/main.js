$(document).ready(function() {
	var contents = [
		new Toolbar.Button('Settings', '/static/images/settings.png').floatRight(),
		new Toolbar.Button('New Project', '/static/images/newproject.png').click(function() {
			window.open('/project/untitled/');
		})
	];

	var toolbar = new Toolbar();
	toolbar.setContents(contents);

	function resize() {
		$('.contents').css({ top: toolbar.height() + 'px' });
	}
	$(window).resize(resize);
	$(window).load(resize);
	resize();
});
