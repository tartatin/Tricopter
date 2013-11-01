window.addEvent('domready', function(){
	
	$('searchChoices').addEvent('click', function(e){
		e.stop();
		
		if($('searchMenu')) $('searchMenu').show();
		
		hoverBG = new Element('div', {'id': 'hoverbg'}).inject($(document.body));
		
		hoverBG.addEvent('click', function(){
			this.destroy();
			if($('searchMenu')) $('searchMenu').hide();
		});
		
		
		$('searchMenu').getElements('a').addEvent('click', function(e){
			
			e.stop();
			
			var className = this.get('class');
		
			if(className == 'wiki'){
				//wiki
				searchText.text.set('html', 'search wiki...')
				action = "http://wiki.bildr.org/index.php/Special:Search";
				method = 'post';
				name = "search";
			}else if(className == 'forum'){
				//forum
				searchText.text.set('html', 'search forum...')
				action = "http://forum.bildr.org//search.php";
				method = 'post';
				name = "keywords";
			}else if(className == 'blog'){
				//blog
				searchText.text.set('html', 'search blog...')
				action = "http://bildr.org/";
				method = 'get';
				name = "s";
			}
						
			$('searchform').set('action', action);
			$('searchform').set('method', method);
			$('searchInput').set('name', name);

			
			if($('searchMenu')) $('searchMenu').hide();
			if($('hoverbg')) $('hoverbg').destroy();
		});
		
		
		
	});	


	
	//overlay search bar
	
	
	var searchText = new OverText($('searchInput'), {textOverride: $('searchChoices').get('data-searchText')});
	
});





