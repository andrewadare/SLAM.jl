$( function() {

  // Initiate a WebSocket connection
  var ws = new WebSocket( 'ws://' + window.location.host );

  // Set up the scene SVG element ASAP to avoid reflowing
  var width = 800;
  var height = 800;
  var xscale = d3.scale.linear()
      .domain([ 0, 100 ])
      .range([ 0, width ]);
  var yscale = d3.scale.linear()
      .domain([ 0, 100 ])
      .range([ height, 0 ]);
  var scene = d3.select( '.scene' )
      .attr( 'width', width )
      .attr( 'height', height );

  function drawWaypoints( data ) {
    scene.selectAll( 'circle' )
        .data( data )
      .enter().append( 'circle' )
        .attr( 'r', 0 ) // px
      .transition()
        .duration( 750 )
        .attr( 'r', 3 ) // px
        .attr( 'cx', function( d ) { return xscale( d.x ); })
        .attr( 'cy', function( d ) { return height - yscale( d.y ); });
  }

  function drawLandmarks( data ) {

    var l = 10; // Edge length of square

    var landmarks = scene.selectAll( 'g' )
        .data(data)
      .enter().append( 'g' )
        .attr( 'transform', function( d ) {
          return 'translate(' + (xscale( d.x ) - l/2) + ',' + (height - yscale( d.y ) + l/2) + ')';
        });

    landmarks.append( 'rect' )
        .attr( 'width', 0 )
        .attr( 'height', 0 )
      .transition()
        .duration( 750 )
        .attr( 'width', l )
        .attr( 'height', l );
  }

  ws.onopen = function( event ) {
    ws.send( JSON.stringify({
      type: 'update',
      text: 'ready',
      id:   0,
      date: Date.now()
    }));

  }

  // Handler for messages received from server
  // https://developer.mozilla.org/en-US/docs/Web/API/WebSockets_API/Writing_WebSocket_client_applications
  ws.onmessage = function( event ) {
    var msg = JSON.parse( event.data );
    switch( msg.type ) {
      case 'waypoints':
        drawWaypoints( msg.data );
        break;
      case 'landmarks':
        drawLandmarks( msg.data );
        break;
    }
  }

  $( '#start' ).click( function() {
  
    // Send message object as a JSON-formatted string.
    ws.send( JSON.stringify({
      type: 'request',
      text: 'get_waypoints',
      id:   1,
      date: Date.now()
    }));

    ws.send( JSON.stringify({
      type: 'request',
      text: 'get_landmarks',
      id:   2,
      date: Date.now()
    }));

  });


});
